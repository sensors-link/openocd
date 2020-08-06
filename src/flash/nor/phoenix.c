/***************************************************************************
 *   Copyright (C) 2020 by Jiazhi Zhang                                    *
 *   Jiazhi Zhang <jiazhi.zhang@fhsjdz.com>                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include "jtag/jtag.h"

#define FLASH_BASE (0x10100000UL)	/*!< ( FLASH   ) Base Address */
#define NVR_BASE (0x10140000UL)		/*!< ( NVR     ) Base Address */
#define EEPROM_BASE (0x10180000UL)	/*!< ( EEPROM  ) Base Address */
#define PAGEBUF_BASE (0x101C0000UL) /*!< ( PAGEBUF ) Base Address */
#define EFC_BASE (0x40000000UL)

#define EFC_CR (EFC_BASE + 0x00)
#define EFC_Tnvs (EFC_BASE + 0x04)
#define EFC_Tprog (EFC_BASE + 0x08)
#define EFC_Tpgs (EFC_BASE + 0x0C)
#define EFC_Trcv (EFC_BASE + 0x10)
#define EFC_Terase (EFC_BASE + 0x14)
#define EFC_WPT (EFC_BASE + 0x18)
#define EFC_OPR (EFC_BASE + 0x1C)
#define EFC_PVEV (EFC_BASE + 0x20)
#define EFC_STS (EFC_BASE + 0x24)

struct phnx_info
{
	uint32_t page_size;
	int num_pages;
	int sector_size;
	int prot_block_size;

	bool probed;
	struct target *target;
};

/**************************** RAW PHOENIX EFC ACTIONS *********************************/
extern int riscv_try_write_memory(struct target *target, target_addr_t address,
								  uint32_t size, uint32_t count, const uint8_t *buffer);

static uint32_t efc_general_op(struct flash_bank *bank, uint32_t op, int offset, uint32_t value, int size)
{
	uint32_t sts;
	if (bank->base == NVR_BASE && op < 4) op |= 0x80; // NVR write enable

	// Reset STS
	target_write_u32(bank->target, EFC_STS, 0xff);
	
	// Set STS operation
	target_write_u32(bank->target, EFC_OPR, 0x00 + op);
	target_write_u32(bank->target, EFC_OPR, 0x70 + op);
	target_write_u32(bank->target, EFC_OPR, 0x90 + op);
	target_write_u32(bank->target, EFC_OPR, 0xC0 + op);

	//load data to specific flash address
	riscv_try_write_memory(bank->target, FLASH_BASE + offset, size, 1, (uint8_t *)&value);

	target_read_u32(bank->target, EFC_STS, &sts);
	int retry = 0;
	while ((sts & 0x01) != 1 && retry < 1000)
	{
		// LOG_INFO("efc_general_op %u STS = %u", op, sts);
		target_read_u32(bank->target, EFC_STS, &sts);
		usleep(10);
		retry++;
	}

	LOG_DEBUG("efc_general_op %u offset=%d, STS=%u, retry=%d", op, offset, sts, retry);
	return sts;
}

#define efc_program_u8(bank, offset, u8) ( efc_general_op(bank, 0, offset, u8, 1) )
#define efc_program_u16(bank, offset, u16) ( efc_general_op(bank, 0, offset, u16, 2) )
#define efc_program_u32(bank, offset, u32) ( efc_general_op(bank, 0, offset, u32, 4) )
#define efc_program_row(bank, offset) ( efc_general_op(bank, 1, offset, 1, 4) )
#define efc_erase_page(bank, offset) ( efc_general_op(bank, 2, offset, 1, 4) )
#define efc_erase_chip(bank, offset) ( efc_general_op(bank, 3, offset, 1, 4) )
#define efc_load_page(bank, offset) ( efc_general_op(bank, 4, offset, 1, 4) )
#define efc_verify_erase_page(bank, offset) ( efc_general_op(bank, 5, offset, 1, 4) )
#define efc_verify_program_page(bank, offset) ( efc_general_op(bank, 6, offset, 1, 4) )

/**************************** OOCD FLASH ACTIONS *********************************/
static int phnx_probe(struct flash_bank *bank)
{
	struct phnx_info *chip = (struct phnx_info *)bank->driver_priv;
	int flash_kb, ram_kb;
	if (chip->probed == true)
		return ERROR_OK;

	// TODO: add a real probe for phoenix chip
	if (bank->base == FLASH_BASE)
	{
		flash_kb = 128, ram_kb = 10;
		chip->sector_size = chip->page_size = 512;
	}
	else if (bank->base == NVR_BASE)
	{
		flash_kb = 4, ram_kb = 10;
		chip->sector_size = chip->page_size = 512;
	}
	else if (bank->base == EEPROM_BASE)
	{
		flash_kb = 1, ram_kb = 10;
		chip->sector_size = chip->page_size = 4;
	}

	chip->num_pages = flash_kb * 1024 / chip->sector_size;
	bank->size = flash_kb * 1024;
	bank->num_sectors = chip->num_pages;
	bank->sectors = alloc_block_array(0, chip->sector_size, bank->num_sectors);
	if (!bank->sectors)
	{
		LOG_ERROR("Couldn't alloc memory for sectors.");
		return ERROR_FAIL;
	}

	/* Done */
	chip->probed = true;

	LOG_INFO("flash: phoenix (%" PRIu32 "KB , %" PRIu32 "KB RAM)", flash_kb, ram_kb);

	return ERROR_OK;
}

static int phnx_protect(struct flash_bank *bank, int set, int first_prot_bl, int last_prot_bl)
{
	LOG_INFO("phnx_protect involked.");
	// TODO:
	return ERROR_OK;
}

static int phnx_erase(struct flash_bank *bank, int first_sect, int last_sect)
{
	struct phnx_info *chip = (struct phnx_info *)bank->driver_priv;

	LOG_INFO("phnx_erase first=%d, last=%d.", first_sect, last_sect);

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}
	
	// Check the chip is probed or not
	if (!chip->probed)
	{
		if (phnx_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (int i = first_sect; i < last_sect + 1; i++)
	{
		if (1 != efc_erase_page(bank, i * chip->sector_size))
		{
			LOG_ERROR("Erase sector %d error !", i);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int phnx_write(struct flash_bank *bank, const uint8_t *buffer,
					  uint32_t offset, uint32_t count)
{
	struct phnx_info *chip = (struct phnx_info *)bank->driver_priv;

	LOG_INFO("phnx_write offset=%u, count=%u.", offset, count);

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed)
	{
		if (phnx_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	uint32_t total = count;
	int start_sector = offset / chip->sector_size;//start sector when writing flash
	int start_sector_offset = offset % chip->sector_size;//the start offset address
	
	/* When the offset address is not zero, loading data to flash offset address */
	if (start_sector_offset != 0)
	{
		if (1 != efc_load_page(bank, start_sector * chip->sector_size))
			return ERROR_FAIL;
		target_write_buffer(bank->target, PAGEBUF_BASE + start_sector_offset,
							MIN((int)count, chip->sector_size - start_sector_offset), buffer);
		/* Erase specific sector before loading data */
		if (1 != efc_erase_page(bank, start_sector * chip->sector_size))
			return ERROR_FAIL;
		/* Loading sector size data to flash */
		if (1 != efc_program_row(bank, start_sector * chip->sector_size))//there need 2 times loading data to specific flash address
			return ERROR_FAIL;
		if (1 != efc_program_row(bank, start_sector * chip->sector_size + chip->sector_size / 2))
			return ERROR_FAIL;
		count -= start_sector_offset;
		buffer += start_sector_offset;
		offset += start_sector_offset;
	}
	
	/* When data size is greater than or equal 1 sector */
	while ((int)count >= chip->sector_size)//loading data to flash sector by sector
	{
		if (1 != efc_erase_page(bank, offset))
			return ERROR_FAIL;
		target_write_buffer(bank->target, PAGEBUF_BASE, chip->sector_size, buffer);
		if (1 != efc_program_row(bank, offset))
			return ERROR_FAIL;
		if (1 != efc_program_row(bank, offset + chip->sector_size / 2))
			return ERROR_FAIL;
		count -= chip->sector_size;
		buffer += chip->sector_size;
		offset += chip->sector_size;
		if ( (offset & 0xfff) == 0) { // print percentage every 4K
			int percentage = ( total - count) * 100 / total;
			LOG_INFO(" ... %d%%", percentage);
		}
	}
	/* When the rest data is less than 1 sector */
	if (count > 0)
	{
		if (1 != efc_load_page(bank, offset))
			return ERROR_FAIL;
		target_write_buffer(bank->target, PAGEBUF_BASE, count, buffer);
		if (1 != efc_erase_page(bank, offset))
			return ERROR_FAIL;
		if (1 != efc_program_row(bank, offset))
			return ERROR_FAIL;
		if (1 != efc_program_row(bank, offset + chip->sector_size / 2))
			return ERROR_FAIL;
	}
	LOG_INFO(" done ...");
	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(phnx_flash_bank_command)
{

	if (bank->base != FLASH_BASE && bank->base != NVR_BASE && bank->base != EEPROM_BASE)
	{
		LOG_ERROR("Address " TARGET_ADDR_FMT PRIx32
				  " invalid bank address (try " TARGET_ADDR_FMT PRIx32 "/" TARGET_ADDR_FMT PRIx32 "/" TARGET_ADDR_FMT PRIx32
				  "[phoenix series] )",
				  (target_addr_t)bank->base, (target_addr_t)FLASH_BASE, (target_addr_t)NVR_BASE, (target_addr_t)EEPROM_BASE);
		return ERROR_FAIL;
	}

	struct phnx_info *chip;
	chip = calloc(1, sizeof(*chip));

	if (!chip)
	{
		LOG_ERROR("No memory for flash bank chip info");
		return ERROR_FAIL;
	}
	chip->target = bank->target;
	chip->probed = false;

	bank->driver_priv = chip;
	return ERROR_OK;
}

COMMAND_HANDLER(phnx_handle_info_command)
{
	struct flash_bank *bank;
	LOG_INFO("phnx_handle_info_command involked.");
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	int bankid = atoi(CMD_ARGV[0]);
	int retval;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, bankid, &bank);
	if (retval != ERROR_OK)
		return retval;

	struct phnx_info *chip = (struct phnx_info *)bank->driver_priv;
	command_print(CMD, "bank %d [%s]: " TARGET_ADDR_FMT PRIx32 ", size=%u, pagesize=%u, npages=%d, %s", 
		bankid, bank->name, bank->base, bank->size, 
		chip->page_size, chip->num_pages, chip->probed? "probed": "notprobed");
	return ERROR_OK;
}

COMMAND_HANDLER(phnx_handle_chip_erase_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	int bankid = atoi(CMD_ARGV[0]);
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, bankid, &bank);
	if (retval != ERROR_OK)
		return retval;

	if (1 != efc_erase_chip(bank, 0))
	{
		command_print(CMD, "chip erase failed");
		return ERROR_FAIL;
	}
	else
	{
		command_print(CMD, "chip erase succeeded");
		return ERROR_OK;
	}
}

static const struct command_registration phoenix_exec_command_handlers[] = {
	{
		.name = "info",
		.handler = phnx_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "Print information about the current bank",
		.usage = "",
	},
	{
		.name = "chip-erase",
		.handler = phnx_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase the entire Flash by using the Chip-Erase feature.",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE};

static const struct command_registration phoenix_command_handlers[] = {
	{
		.name = "phoenix",
		.mode = COMMAND_ANY,
		.help = "phoenix flash command group",
		.usage = "",
		.chain = phoenix_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE};

struct flash_driver phoenix_flash = {
	.name = "phoenix",
	.commands = phoenix_command_handlers,
	.flash_bank_command = phnx_flash_bank_command,
	.erase = phnx_erase,
	.protect = phnx_protect,
	.write = phnx_write,
	.read = default_flash_read,
	.probe = phnx_probe,
	.auto_probe = phnx_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

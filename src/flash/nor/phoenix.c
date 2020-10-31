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

	struct timeval now;
	gettimeofday(&now, NULL);
	// LOG_INFO("%d.%d efc_general_op %u offset=%d, STS=%u, retry=%d", (int)now.tv_sec, (int) now.tv_usec / 1000, op, offset, sts, retry);
	LOG_DEBUG("%d.%d efc_general_op %u offset=%d, STS=%u, retry=%d", (int)now.tv_sec, (int) now.tv_usec / 1000, op, offset, sts, retry);
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

	// Bypass chip erase, while phnx_batch_write will erase page automatically
	// for (int i = first_sect; i < last_sect + 1; i++)
	// {
	// 	if (1 != efc_erase_page(bank, i * chip->sector_size))
	// 	{
	// 		LOG_ERROR("Erase sector %d error !", i);
	// 		return ERROR_FAIL;
	// 	}
	// }
	return ERROR_OK;
}


static int phnx_batch_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct phnx_info *chip = (struct phnx_info *)bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 8192;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[3];
	int retval = ERROR_OK;

	LOG_INFO("phnx_batch_write offset=%u, count=%u.", offset, count);
	if (offset % chip->sector_size != 0)
	{
		LOG_ERROR("offset not aligned by sector size %d", chip->sector_size);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
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

	static const uint8_t phoenix_flash_write_code[] = {
/* Autogenerated with bin2char.sh */
0x11,0xa0,0x02,0x90,0x97,0x21,0x00,0x00,0x93,0x81,0xa1,0x9c,0x37,0x21,0x00,0x20,
0x11,0x20,0xc5,0xbf,0x31,0x11,0x22,0xc8,0x40,0x08,0x23,0x2a,0xa4,0xfe,0x23,0x28,
0xb4,0xfe,0x23,0x26,0xc4,0xfe,0x49,0xa2,0xb7,0x07,0x00,0x40,0x93,0x87,0x47,0x02,
0x13,0x07,0xf0,0x0f,0x98,0xc3,0xb7,0x07,0x00,0x40,0xf1,0x07,0x09,0x47,0x98,0xc3,
0xb7,0x07,0x00,0x40,0xf1,0x07,0x13,0x07,0x20,0x07,0x98,0xc3,0xb7,0x07,0x00,0x40,
0xf1,0x07,0x13,0x07,0x20,0x09,0x98,0xc3,0xb7,0x07,0x00,0x40,0xf1,0x07,0x13,0x07,
0x20,0x0c,0x98,0xc3,0x03,0x27,0x44,0xff,0xb7,0x07,0x10,0x10,0xba,0x97,0x3e,0x87,
0x85,0x47,0x1c,0xc3,0xb7,0x07,0x00,0x40,0x93,0x87,0x47,0x02,0x98,0x43,0x85,0x47,
0x63,0x19,0xf7,0x12,0x23,0x2c,0x04,0xfe,0x25,0xa0,0x03,0x27,0x04,0xff,0x83,0x27,
0x84,0xff,0xba,0x97,0xbe,0x86,0x03,0x27,0x84,0xff,0xb7,0x07,0x1c,0x10,0xba,0x97,
0x3e,0x87,0x9c,0x42,0x1c,0xc3,0x83,0x27,0x84,0xff,0x91,0x07,0x23,0x2c,0xf4,0xfe,
0x03,0x27,0x84,0xff,0x93,0x07,0xf0,0x1f,0x63,0xc8,0xe7,0x00,0x03,0x27,0x84,0xff,
0x83,0x27,0xc4,0xfe,0xe3,0x43,0xf7,0xfc,0xb7,0x07,0x00,0x40,0x93,0x87,0x47,0x02,
0x13,0x07,0xf0,0x0f,0x98,0xc3,0xb7,0x07,0x00,0x40,0xf1,0x07,0x05,0x47,0x98,0xc3,
0xb7,0x07,0x00,0x40,0xf1,0x07,0x13,0x07,0x10,0x07,0x98,0xc3,0xb7,0x07,0x00,0x40,
0xf1,0x07,0x13,0x07,0x10,0x09,0x98,0xc3,0xb7,0x07,0x00,0x40,0xf1,0x07,0x13,0x07,
0x10,0x0c,0x98,0xc3,0x03,0x27,0x44,0xff,0xb7,0x07,0x10,0x10,0xba,0x97,0x3e,0x87,
0x85,0x47,0x1c,0xc3,0xb7,0x07,0x00,0x40,0x93,0x87,0x47,0x02,0x98,0x43,0x85,0x47,
0x63,0x1b,0xf7,0x08,0xb7,0x07,0x00,0x40,0x93,0x87,0x47,0x02,0x13,0x07,0xf0,0x0f,
0x98,0xc3,0xb7,0x07,0x00,0x40,0xf1,0x07,0x05,0x47,0x98,0xc3,0xb7,0x07,0x00,0x40,
0xf1,0x07,0x13,0x07,0x10,0x07,0x98,0xc3,0xb7,0x07,0x00,0x40,0xf1,0x07,0x13,0x07,
0x10,0x09,0x98,0xc3,0xb7,0x07,0x00,0x40,0xf1,0x07,0x13,0x07,0x10,0x0c,0x98,0xc3,
0x03,0x27,0x44,0xff,0xb7,0x07,0x10,0x10,0x93,0x87,0x07,0x10,0xba,0x97,0x3e,0x87,
0x85,0x47,0x1c,0xc3,0xb7,0x07,0x00,0x40,0x93,0x87,0x47,0x02,0x98,0x43,0x85,0x47,
0x63,0x1d,0xf7,0x02,0x83,0x27,0x44,0xff,0x93,0x87,0x07,0x20,0x23,0x2a,0xf4,0xfe,
0x83,0x27,0x04,0xff,0x93,0x87,0x07,0x20,0x23,0x28,0xf4,0xfe,0x83,0x27,0xc4,0xfe,
0x93,0x87,0x07,0xe0,0x23,0x26,0xf4,0xfe,0x83,0x27,0xc4,0xfe,0xe3,0x4e,0xf0,0xe6,
0x31,0xa0,0x01,0x00,0x21,0xa0,0x01,0x00,0x11,0xa0,0x01,0x00,0xb7,0x07,0x00,0x40,
0x93,0x87,0x47,0x02,0x9c,0x43,0x3e,0x85,0x42,0x44,0x51,0x01,0x82,0x80,
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(phoenix_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(phoenix_flash_write_code), phoenix_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "a0", 32, PARAM_IN_OUT);	/* flash offset */
	init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);	/* buffer address */
	init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);	/* byte count */

	// LOG_INFO("buffer_size = %d", buffer_size);
	int total = count;
	while (count > 0) {
		uint32_t run_bytes = count > buffer_size? buffer_size: count;

		/* Write data to fifo */
		retval = target_write_buffer(target, source->address, run_bytes, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, offset);
		buf_set_u32(reg_params[1].value, 0, 32, source->address);
		buf_set_u32(reg_params[2].value, 0, 32, run_bytes);

		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
				write_algorithm->address, write_algorithm->address+2,
				100000, NULL);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d",
					write_algorithm->address, retval);
			break;
		}

		retval = buf_get_u32(reg_params[0].value,0,32);
		if (retval != 1) {
			LOG_ERROR("flash write failed, retval=%x",(uint32_t) retval);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		} else {
			retval = ERROR_OK;
		}

		/* Update counters and wrap write pointer */
		buffer += run_bytes;
		offset += run_bytes;
		count -= run_bytes;
		int percentage = ( total - count) * 100 / total;
		LOG_INFO(" ... %d%%", percentage);
	}

	if (retval == ERROR_OK ) LOG_INFO(" done ...");
	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return retval;
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

COMMAND_HANDLER(phnx_handle_softreset_command)
{
	struct flash_bank *bank;
	int retval;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	// Unlock PMU
	target_write_u32(bank->target, 0x40012C18, 0xC3);
	target_write_u32(bank->target, 0x40012C18, 0x3C);
	// Soft Reset
	retval = 1;
	riscv_try_write_memory(bank->target, 0x40012C08, 4, 1, (uint8_t *)&retval);
	return ERROR_OK;
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
	{
		.name = "softreset",
		.handler = phnx_handle_softreset_command,
		.mode = COMMAND_EXEC,
		.help = "Soft reset (chip reset) phoenix SoC.",
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
	.write = phnx_batch_write,
	.read = default_flash_read,
	.probe = phnx_probe,
	.auto_probe = phnx_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

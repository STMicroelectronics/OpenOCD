/***************************************************************************
 *   Copyright (C) 2016 - 2018 by Andreas Bolsch                           *
 *   andreas.bolsch@mni.thm.de                                             *
 *                                                                         *
 *   Copyright (C) 2010 by Antonio Borneo                                  *
 *   borneo.antonio@gmail.com                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
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

/* STM QuadSPI (QSPI) and OctoSPI (OCTOSPI) controller are SPI bus controllers
 * specifically designed for SPI memories.
 * Two working modes are available:
 * - indirect mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus.
 * - memory mapped mode: the SPI is under QSPI/OCTOSPI control. Memory content
 *   is directly accessible in CPU memory space. CPU can read and execute from
 *   memory (but not write to) */

/* ATTENTION:
 * To have flash mapped in CPU memory space, the QSPI/OCTOSPI controller
 * has to be in "memory mapped mode". This requires following constraints:
 * 1) The command "reset init" has to initialize QSPI/OCTOSPI controller and put
 *    it in memory mapped mode;
 * 2) every command in this file has to return to prompt in memory mapped mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include "stmqspi.h"

#define READ_REG(a)											\
({															\
	uint32_t __v;											\
															\
	retval = target_read_u32(target, io_base + (a), &__v);	\
	(retval == ERROR_OK) ? __v : 0x0;						\
})

/* saved mode settings */
#define QSPI_MODE (stmqspi_info->saved_ccr & \
	(0xF0000000U | QSPI_DCYC_MASK | QSPI_4LINE_MODE | QSPI_ALTB_MODE | QSPI_ADDR4))

/* QSPI_CCR for various commands, these never use dummy cycles nor alternate bytes */
#define	QSPI_CCR_READ_STATUS \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB) | \
	(QSPI_READ_MODE | SPIFLASH_READ_STATUS))

#define	QSPI_CCR_READ_ID \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB) | \
	(QSPI_READ_MODE | SPIFLASH_READ_ID))

#define QSPI_CCR_WRITE_ENABLE \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB & QSPI_NO_DATA) | \
	(QSPI_WRITE_MODE | SPIFLASH_WRITE_ENABLE))

#define QSPI_CCR_SECTOR_ERASE \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB & QSPI_NO_DATA) | \
	(QSPI_WRITE_MODE | stmqspi_info->dev.erase_cmd))

#define QSPI_CCR_MASS_ERASE \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB & QSPI_NO_DATA) | \
	(QSPI_WRITE_MODE | stmqspi_info->dev.chip_erase_cmd))

#define QSPI_CCR_PAGE_PROG \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB) | \
	(QSPI_WRITE_MODE | stmqspi_info->dev.pprog_cmd))

/* saved mode settings */
#define OCTOSPI_MODE (stmqspi_info->saved_cr & 0xCFFFFFFF)

#define OPI_MODE ((stmqspi_info->saved_ccr & OCTOSPI_ISIZE_MASK) != 0)

#define OCTOSPI_MODE_CCR (stmqspi_info->saved_ccr & \
	(0xF0000000U | OCTOSPI_8LINE_MODE | OCTOSPI_ALTB_MODE | OCTOSPI_ADDR4))

/* OCTOSPI_CCR for various commands, these never use alternate bytes	*
 * for READ_STATUS and READ_ID, 4-byte address 0 and 4 dummy bytes must	*
 * be sent in OPI mode													*/
#define OPI_DUMMY 4U

#define	OCTOSPI_CCR_READ_STATUS \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_DDTR & \
	(OPI_MODE ? ~0U : OCTOSPI_NO_ADDR) & OCTOSPI_NO_ALTB))

#define	OCTOSPI_CCR_READ_ID \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_DDTR & \
	(OPI_MODE ? ~0U : OCTOSPI_NO_ADDR) & OCTOSPI_NO_ALTB))

#define OCTOSPI_CCR_WRITE_ENABLE \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_ADDR & OCTOSPI_NO_ALTB & OCTOSPI_NO_DATA))

#define OCTOSPI_CCR_SECTOR_ERASE \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_ALTB & OCTOSPI_NO_DATA))

#define OCTOSPI_CCR_MASS_ERASE \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_ADDR & OCTOSPI_NO_ALTB & OCTOSPI_NO_DATA))

#define OCTOSPI_CCR_PAGE_PROG \
	((OCTOSPI_MODE_CCR & QSPI_NO_ALTB))

#define SPI_ADSIZE (((stmqspi_info->saved_ccr >> SPI_ADSIZE_POS) & 0x3) + 1)

#define OPI_CMD(cmd) (OPI_MODE ? ((((uint16_t) cmd)<<8) | (~cmd & 0xFF)) : cmd)

#define OCTOSPI_CMD(mode, ccr, ir)										\
({																		\
	retval = target_write_u32(target, io_base + OCTOSPI_CR,				\
		OCTOSPI_MODE | mode);											\
	if (retval == ERROR_OK)												\
		retval = target_write_u32(target, io_base + OCTOSPI_TCR,		\
			(stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK) |			\
			((OPI_MODE && (mode == OCTOSPI_READ_MODE)) ?				\
			(OPI_DUMMY<<OCTOSPI_DCYC_POS) : 0));						\
	if (retval == ERROR_OK)												\
		retval = target_write_u32(target, io_base + OCTOSPI_CCR, ccr);	\
	if (retval == ERROR_OK)												\
		retval = target_write_u32(target, io_base + OCTOSPI_IR,			\
			OPI_CMD(ir));												\
	retval;																\
})

/* convert uint32_t into 4 uint8_t in little endian byte order */
static inline uint32_t h_to_le_32(uint32_t val)
{
	union {
		uint32_t word;
		uint8_t byte[sizeof(uint32_t)];
	} res;

	res.byte[0] = val & 0xFF;
	res.byte[1] = (val>>8) & 0xFF;
	res.byte[2] = (val>>16) & 0xFF;
	res.byte[3] = (val>>24) & 0xFF;

	return res.word;
}

/* Timeout in ms */
#define SPI_CMD_TIMEOUT			(100)
#define SPI_PROBE_TIMEOUT		(100)
#define SPI_MAX_TIMEOUT			(2000)
#define SPI_MASS_ERASE_TIMEOUT	(400000)

struct sector_info {
	uint32_t address;
	uint32_t size;
	uint32_t result;
};

struct stmqspi_flash_bank {
	int probed;
	char devname[32];
	bool octo;
	struct flash_device dev;
	uint32_t io_base;
	uint32_t saved_cr;	/* FSEL and DFM bit mask in QUADSPI_CR / OCTOSPI_CR */
	uint32_t saved_ccr;
	uint32_t saved_tcr;	/* only for OCTOSPI */
	uint32_t saved_ir;	/* only for OCTOSPI */
};

FLASH_BANK_COMMAND_HANDLER(stmqspi_flash_bank_command)
{
	struct stmqspi_flash_bank *stmqspi_info;
	uint32_t io_base;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], bank->base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], io_base);

	stmqspi_info = malloc(sizeof(struct stmqspi_flash_bank));
	if (stmqspi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = stmqspi_info;
	stmqspi_info->probed = 0;
	stmqspi_info->io_base = io_base;

	return ERROR_OK;
}

/* Poll busy flag */
/* timeout in ms */
static int poll_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	int retval;
	long long endtime;

	if ((READ_REG(SPI_SR) & (1<<SPI_BUSY)) == 0)
		return ERROR_OK;

	endtime = timeval_ms() + timeout;
	do {
		alive_sleep(1);
		if ((READ_REG(SPI_SR) & (1<<SPI_BUSY)) == 0) {
			/* Clear transmit finished flag */
			retval = target_write_u32(target, io_base + SPI_FCR, (1<<SPI_TCF));
			return retval;
		} else
			LOG_DEBUG("busy: 0x%08X", READ_REG(SPI_SR));
	} while (timeval_ms() < endtime);

	LOG_ERROR("Timeout while polling BUSY");
	return ERROR_FLASH_OPERATION_FAILED;
}

/* Set to memory-mapped mode, e. g. after an error */
static int set_mm_mode(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	int retval;

	/* Reset Address register bits 0 and 1, see various errata sheets */
	retval = target_write_u32(target, io_base + SPI_AR, 0x0);
	if (retval != ERROR_OK)
		return retval;

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | (1<<SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Finally switch to memory mapped mode */
	if (IS_OCTOSPI) {
		retval = target_write_u32(target, io_base + OCTOSPI_CR,
			OCTOSPI_MODE | OCTOSPI_MM_MODE);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, io_base + OCTOSPI_CCR,
				stmqspi_info->saved_ccr);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, io_base + OCTOSPI_TCR,
				stmqspi_info->saved_tcr);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, io_base + OCTOSPI_IR,
				stmqspi_info->saved_ir);
	} else {
		retval = target_write_u32(target, io_base + QSPI_CCR, stmqspi_info->saved_ccr);
	}
	return retval;
}

/* Read the status register of the external SPI flash chip(s). */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint8_t data;
	int retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Read one byte per chip */
	retval = target_write_u32(target, io_base + SPI_DLR,
		(stmqspi_info->saved_cr & (1<<SPI_DUAL_FLASH)) ? 1 : 0);
	if (retval != ERROR_OK)
		goto err;

	/* Read status */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_READ_MODE, OCTOSPI_CCR_READ_STATUS, SPIFLASH_READ_STATUS);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_READ_STATUS);

	/* Dummy address 0, only required for 8-line mode */
	retval = target_write_u32(target, io_base + SPI_AR, 0);
	if (retval != ERROR_OK)
		goto err;

	if (retval != ERROR_OK)
		goto err;

	*status = 0;

	if ((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (1<<SPI_FSEL_FLASH)) {
		/* get status of flash 1 in dual mode or flash 1 only mode */
		retval = target_read_u8(target, io_base + SPI_DR, &data);
		if (retval != ERROR_OK)
			goto err;
		*status |= data;
	}

	if ((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (0<<SPI_FSEL_FLASH)) {
		/* get status of flash 2 in dual mode or flash 2 only mode */
		retval = target_read_u8(target, io_base + SPI_DR, &data);
		if (retval != ERROR_OK)
			goto err;
		*status |= data << 8;
	}

	LOG_DEBUG("flash status regs: 0x%04" PRIx16, (uint16_t) *status);

err:
	return retval;
}

/* check for WIP (write in progress) bit(s) in status register(s) */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval;
	long long endtime;

	endtime = timeval_ms() + timeout;
	do {
		/* Read flash status register(s) */
		retval = read_status_reg(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & ((SPIFLASH_BSY_BIT << 8) || SPIFLASH_BSY_BIT)) == 0)
			return retval;
		alive_sleep(25);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

/* Send "write enable" command to SPI flash chip(s). */
static int qspi_write_enable(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint32_t status;
	int retval;

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | (1<<SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Send write enable command */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE, OCTOSPI_CCR_WRITE_ENABLE, SPIFLASH_WRITE_ENABLE);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_WRITE_ENABLE);
	if (retval != ERROR_OK)
		goto err;

	/* Dummy address 0, only required for 8-line mode */
	retval = target_write_u32(target, io_base + SPI_AR, 0);
	if (retval != ERROR_OK)
		goto err;

	/* Read flash status register */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		goto err;

	/* Check write enabled for flash 1 */
	if (((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (1<<SPI_FSEL_FLASH)) && ((status & SPIFLASH_WE_BIT) == 0)) {
		LOG_ERROR("Cannot write enable flash1. Status=0x%02" PRIx8,
				(uint8_t)(status & 0xFF));
		return ERROR_FAIL;
	}

	/* Check write enabled for flash 2 */
	if (((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (0<<SPI_FSEL_FLASH)) && (((status >> 8) & SPIFLASH_WE_BIT) == 0)) {
		LOG_ERROR("Cannot write enable flash2. Status=0x%02" PRIx8,
				(uint8_t) ((status >> 8) & 0xFF));
		return ERROR_FAIL;
	}

err:
	return retval;
}

COMMAND_HANDLER(stmqspi_handle_mass_erase_command)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct stmqspi_flash_bank *stmqspi_info;
	struct duration bench;
	uint32_t io_base, status;
	int retval, sector;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stmqspi_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (stmqspi_info->dev.chip_erase_cmd == 0x00) {
		LOG_ERROR("Mass erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	for (sector = 0; sector <= bank->num_sectors; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	io_base = stmqspi_info->io_base;
	duration_start(&bench);

	retval = qspi_write_enable(bank);
	if (retval != ERROR_OK)
		goto err;

	/* Send Mass Erase command */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE, OCTOSPI_CCR_MASS_ERASE,
			stmqspi_info->dev.chip_erase_cmd);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_MASS_ERASE);
	if (retval != ERROR_OK)
		goto err;

	/* Read flash status register(s) */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		goto err;

	/* Check for command in progress for flash 1 */
	if (((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (1<<SPI_FSEL_FLASH)) && ((status & SPIFLASH_BSY_BIT) == 0)) {
		LOG_ERROR("Mass erase command not accepted by flash1. Status=0x%04" PRIx16, (uint16_t) status);
		retval = ERROR_FAIL;
		goto err;
	}

	/* Check for command in progress for flash 2 */
	if (((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (0<<SPI_FSEL_FLASH)) && (((status >> 8) & SPIFLASH_BSY_BIT) == 0)) {
		LOG_ERROR("Mass erase command not accepted by flash2. Status=0x%04" PRIx16, (uint16_t) status);
		retval = ERROR_FAIL;
		goto err;
	}

	/* Poll WIP for end of self timed Sector Erase cycle */
	retval = wait_till_ready(bank, SPI_MASS_ERASE_TIMEOUT);

	duration_measure(&bench);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (sector = 0; sector < bank->num_sectors; sector++)
			bank->sectors[sector].is_erased = 1;

		command_print(CMD_CTX, "stmqspi mass erase completed in"
			" %fs (%0.3f KiB/s)", duration_elapsed(&bench),
			duration_kbps(&bench, bank->size));
	} else {
		command_print(CMD_CTX, "stmqspi mass erase failed after %fs",
			duration_elapsed(&bench));
	}

err:
	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int log2u(uint32_t word)
{
	int result;

	for (result = 0; (unsigned int) result < sizeof(unsigned long) * 8; result++)
		if (word == (1UL<<result))
			return result;

	return -1;
}

COMMAND_HANDLER(stmqspi_handle_set)
{
	struct flash_bank *bank = NULL;
	struct target *target = NULL;
	struct stmqspi_flash_bank *stmqspi_info = NULL;
	struct flash_sector *sectors = NULL;
	uint32_t io_base, temp;
	unsigned int index = 2;
	int dual, fsize, retval;

	LOG_DEBUG("%s", __func__);

	if ((CMD_ARGC < 6) || (CMD_ARGC > 10))
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	stmqspi_info = bank->driver_priv;

	/* invalidate all old info */
	if (stmqspi_info->probed)
		free(bank->sectors);
	bank->size = 0;
	bank->num_sectors = 0;
	bank->sectors = NULL;
	stmqspi_info->probed = 0;
	memset(&stmqspi_info->dev, 0, sizeof(stmqspi_info->dev));
	stmqspi_info->dev.name = "unknown";

	strncpy(stmqspi_info->devname, CMD_ARGV[1], sizeof(stmqspi_info->devname) - 1);
	stmqspi_info->devname[sizeof(stmqspi_info->devname) - 1] = '\0';

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
	stmqspi_info->dev.size_in_bytes = temp;
	if (log2u(stmqspi_info->dev.size_in_bytes) < 8) {
		command_print(CMD_CTX, "stmqspi: device size must be 2^n with n >= 8");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], stmqspi_info->dev.pagesize);
	if ((log2u(stmqspi_info->dev.pagesize) > log2u(stmqspi_info->dev.size_in_bytes)) ||
		(log2u(stmqspi_info->dev.pagesize) < 0)) {
		command_print(CMD_CTX, "stmqspi: page size must be 2^n and <= device size");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.read_cmd);
	if ((stmqspi_info->dev.read_cmd != 0x03) &&
		(stmqspi_info->dev.read_cmd != 0x13)) {
		command_print(CMD_CTX, "stmqspi: only 0x03/0x13 READ cmd allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.fread_cmd);
	if ((stmqspi_info->dev.fread_cmd != 0x00) &&
		(stmqspi_info->dev.fread_cmd != 0x0B) &&
		(stmqspi_info->dev.fread_cmd != 0x0C) &&
		(stmqspi_info->dev.fread_cmd != 0x3B) &&
		(stmqspi_info->dev.fread_cmd != 0x3C) &&
		(stmqspi_info->dev.fread_cmd != 0x6B) &&
		(stmqspi_info->dev.fread_cmd != 0x6C) &&
		(stmqspi_info->dev.fread_cmd != 0xBB) &&
		(stmqspi_info->dev.fread_cmd != 0xBC) &&
		(stmqspi_info->dev.fread_cmd != 0xEB) &&
		(stmqspi_info->dev.fread_cmd != 0xEC)) {
		command_print(CMD_CTX, "stmqspi: only 0x0B/0x0C/0x3B/0x3C/"
			"0x6B/0x6C/0xBB/0xBC/0xEB/0xEC FAST READ allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.pprog_cmd);
	if ((stmqspi_info->dev.pprog_cmd != 0x02) &&
		(stmqspi_info->dev.pprog_cmd != 0x12) &&
		(stmqspi_info->dev.pprog_cmd != 0x32)) {
		command_print(CMD_CTX, "stmqspi: only 0x02/0x12/0x32 PPRG cmd allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (index < CMD_ARGC)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.chip_erase_cmd);
	else
		stmqspi_info->dev.chip_erase_cmd = 0x00;

	if (index < CMD_ARGC) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
		stmqspi_info->dev.sectorsize = temp;
		if ((log2u(stmqspi_info->dev.sectorsize) > log2u(stmqspi_info->dev.size_in_bytes)) ||
			(log2u(stmqspi_info->dev.sectorsize) < 0)) {
			command_print(CMD_CTX, "stmqspi: sector size must be 2^n and <= device size");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (index < CMD_ARGC)
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.erase_cmd);
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	} else {
		/* no sector size / sector erase cmd given, treat whole bank as a single sector */
		stmqspi_info->dev.erase_cmd = 0x00;
		stmqspi_info->dev.sectorsize = stmqspi_info->dev.size_in_bytes;
	}

	dual = (stmqspi_info->saved_cr & (1<<SPI_DUAL_FLASH)) ? 1 : 0;

	/* set correct size value */
	bank->size = stmqspi_info->dev.size_in_bytes << dual;

	io_base = stmqspi_info->io_base;
	fsize = (READ_REG(SPI_DCR)>>SPI_FSIZE_POS) & ((1U<<SPI_FSIZE_LEN) - 1);
	LOG_DEBUG("FSIZE = 0x%04x", fsize);
	if (bank->size != (1U<<(fsize + 1)))
		LOG_WARNING("FSIZE field in QSPI_DCR(1) doesn't match actual capacity.");

	if (stmqspi_info->dev.sectorsize != 0) {
		/* create and fill sectors array */
		bank->num_sectors =
			stmqspi_info->dev.size_in_bytes / stmqspi_info->dev.sectorsize;
		sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		if (sectors == NULL) {
			LOG_ERROR("not enough memory");
			return ERROR_FAIL;
		}

		for (int sector = 0; sector < bank->num_sectors; sector++) {
			sectors[sector].offset = sector * (stmqspi_info->dev.sectorsize << dual);
			sectors[sector].size = (stmqspi_info->dev.sectorsize << dual);
			sectors[sector].is_erased = -1;
			sectors[sector].is_protected = 0;
		}
	} else {
		/* treat whole flash as single sector */
		bank->num_sectors = 1;
		sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		if (sectors == NULL) {
			LOG_ERROR("not enough memory");
			return ERROR_FAIL;
		}

		sectors[0].offset = 0;
		sectors[0].size = stmqspi_info->dev.size_in_bytes << dual;
		sectors[0].is_erased = -1;
		sectors[0].is_protected = 0;
	}

	bank->sectors = sectors;
	stmqspi_info->dev.name = stmqspi_info->devname;
	LOG_INFO("flash \'%s\' id = unknown\nchip size = %lukbytes,"
		" bank size = %lukbytes", stmqspi_info->dev.name,
		stmqspi_info->dev.size_in_bytes>>10,
		(stmqspi_info->dev.size_in_bytes>>10)<<dual);

	stmqspi_info->probed = 1;

	return ERROR_OK;
}

COMMAND_HANDLER(stmqspi_handle_cmd)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct stmqspi_flash_bank *stmqspi_info = NULL;
	uint32_t io_base, addr;
	uint8_t num_write, num_read, cmd_byte, data;
	unsigned int count;
	const unsigned int max = 21;
	char temp[4], output[(2 + max + 256) * 3 + 8];
	int retval;

	LOG_DEBUG("%s", __func__);

	if ((CMD_ARGC < 3) || (CMD_ARGC > max + 3))
		return ERROR_COMMAND_SYNTAX_ERROR;

	num_write = CMD_ARGC - 2;
	if (num_write > max) {
		LOG_ERROR("at most %d bytes may be send", max);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	stmqspi_info = bank->driver_priv;
	io_base = stmqspi_info->io_base;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], num_read);
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2], cmd_byte);

	if (stmqspi_info->saved_cr & (1<<SPI_DUAL_FLASH)) {
		if ((num_write & 1) == 0) {
			LOG_ERROR("number of data bytes to write must be even in dual mode");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if ((num_read & 1) != 0) {
			LOG_ERROR("number of bytes to read must be even in dual mode");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (((num_write < 1) || (num_write > 5)) && (num_read > 0)) {
		LOG_ERROR("one cmd and up to four addr bytes must be send when reading");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | (1<<SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* send command byte */
	snprintf(output, sizeof(output), "spicmd: %02x ", cmd_byte);
	if (num_read == 0) {
		/* write, send cmd byte */
		retval = target_write_u32(target, io_base + SPI_DLR, ((uint32_t) num_write) - 2);
		if (retval != ERROR_OK)
			goto err;

		if (IS_OCTOSPI)
			retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE,
				(OCTOSPI_MODE_CCR & OCTOSPI_NO_ALTB & OCTOSPI_NO_ADDR &
				((num_write == 1) ? OCTOSPI_NO_DATA : ~0U)), cmd_byte);
		else
			retval = target_write_u32(target, io_base + QSPI_CCR,
				(QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB & QSPI_NO_ADDR &
				((num_write == 1) ? QSPI_NO_DATA : ~0U)) |
				(QSPI_WRITE_MODE | cmd_byte));
		if (retval != ERROR_OK)
			goto err;

		/* send additional data bytes */
		for (count = 3; count < CMD_ARGC; count++) {
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[count], data);
			snprintf(temp, sizeof(temp), "%02x ", data);
			retval = target_write_u8(target, io_base + SPI_DR, data); \
			if (retval != ERROR_OK)
				goto err;
			strncat(output, temp, sizeof(output) - strlen(output) - 1);
		}
		strncat(output, "-> ", sizeof(output) - strlen(output) - 1);
	} else {
		/* read, pack additional bytes into address */
		addr = 0;
		for (count = 3; count < CMD_ARGC; count++) {
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[count], data);
			snprintf(temp, sizeof(temp), "%02x ", data);
			addr = (addr << 8) | data;
			strncat(output, temp, sizeof(output) - strlen(output) - 1);
		}
		strncat(output, "-> ", sizeof(output) - strlen(output) - 1);

		/* send cmd byte, if ADMODE indicates no address, this already triggers command */
		retval = target_write_u32(target, io_base + SPI_DLR, ((uint32_t) num_read) - 1);
		if (retval != ERROR_OK)
			goto err;
		if (IS_OCTOSPI)
			retval = OCTOSPI_CMD(OCTOSPI_READ_MODE,
				(OCTOSPI_MODE_CCR & OCTOSPI_NO_DDTR & OCTOSPI_NO_ALTB &
				((num_write == 1) ? OCTOSPI_NO_ADDR : ~0U)) |
				(((num_write - 2) & 0x3U)<<SPI_ADSIZE_POS), cmd_byte);
		else
			retval = target_write_u32(target, io_base + QSPI_CCR,
				(QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB & ((num_write == 1) ? QSPI_NO_ADDR : ~0U)) |
				((QSPI_READ_MODE | (((num_write - 2) & 0x3U)<<SPI_ADSIZE_POS) | cmd_byte)));
		if (retval != ERROR_OK)
			goto err;

		if (num_write > 1) {
			/* if ADMODE indicates address required, only the write to AR triggers command */
			retval = target_write_u32(target, io_base + SPI_AR, addr);
			if (retval != ERROR_OK)
				goto err;
		}

		/* read response bytes */
		for ( ; num_read > 0; num_read--) {
			retval = target_read_u8(target, io_base + SPI_DR, &data);
			if (retval != ERROR_OK)
				goto err;
			snprintf(temp, sizeof(temp), "%02x ", data);
			strncat(output, temp, sizeof(output) - strlen(output) - 1);
		}
	}
	command_print(CMD_CTX, "%s", output);

err:
	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);
	return retval;
}

static int qspi_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint32_t status;
	int retval;

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | (1<<SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	retval = qspi_write_enable(bank);
	if (retval != ERROR_OK)
		goto err;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Send Sector Erase command */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE, OCTOSPI_CCR_SECTOR_ERASE,
			stmqspi_info->dev.erase_cmd);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_SECTOR_ERASE);
	if (retval != ERROR_OK)
		goto err;

	/* Address is sector offset, this write initiates command transmission */
	retval = target_write_u32(target, io_base + SPI_AR, bank->sectors[sector].offset);
	if (retval != ERROR_OK)
		goto err;

	/* Read flash status register(s) */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		goto err;

	LOG_DEBUG("erase status regs: 0x%04" PRIx16, (uint16_t) status);

	/* Check for command in progress for flash 1 */
	if (((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (1<<SPI_FSEL_FLASH)) && ((status & SPIFLASH_BSY_BIT) == 0)) {
		LOG_ERROR("Sector erase command not accepted by flash1. Status=0x%04" PRIx16, (uint16_t) status);
		retval = ERROR_FAIL;
		goto err;
	}

	/* Check for command in progress for flash 2 */
	if (((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) | (1<<SPI_FSEL_FLASH)))
		!= (0<<SPI_FSEL_FLASH)) && (((status >> 8) & SPIFLASH_BSY_BIT) == 0)) {
		LOG_ERROR("Sector erase command not accepted by flash2. Status=0x%04" PRIx16, (uint16_t) status);
		retval = ERROR_FAIL;
		goto err;
	}

	/* Poll WIP for end of self timed Sector Erase cycle */
	retval = wait_till_ready(bank, SPI_MAX_TIMEOUT);

	/* Erase takes a long time, so some sort of progress message is a good idea */
	LOG_DEBUG("sector %4d erased", sector);

err:
	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int stmqspi_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (stmqspi_info->dev.erase_cmd == 0x00) {
		LOG_ERROR("Sector erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	for (sector = first; sector <= last; sector++) {
		retval = qspi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	if (retval != ERROR_OK)
		LOG_ERROR("Flash sector_erase failed on sector %d", sector);

	return retval;
}

static int stmqspi_protect(struct flash_bank *bank, int set,
	int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int qspi_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	struct working_area *write_algorithm;
	uint32_t pagesize, fifo_start, fifosize, maxsize, exit_point;
	uint32_t write_code_size, remaining;
	const uint8_t *write_code = NULL;
	int dual, retval = ERROR_OK;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " len=0x%08" PRIx32,
		__func__, offset, count);

	/* see contrib/loaders/flash/stmqspi_write.S for src */
	static const uint8_t stmqspi_write_code[] = {
#include "../../../contrib/loaders/flash/stmqspi/stmqspi_write.inc"
	};

	/* see contrib/loaders/flash/stmoctospi_write.S for src */
	static const uint8_t stmoctospi_write_code[] = {
#include "../../../contrib/loaders/flash/stmqspi/stmoctospi_write.inc"
	};

	/* This will overlay the last 12 words of stmqspi/stmoctospi_write_code in target */
	uint32_t ccr_buffer[][4] = {
		{
			h_to_le_32(OCTOSPI_MODE | OCTOSPI_READ_MODE),
			h_to_le_32(IS_OCTOSPI ? OCTOSPI_CCR_READ_STATUS : QSPI_CCR_READ_STATUS),
			h_to_le_32((stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK) |
						(OPI_MODE ? (OPI_DUMMY<<OCTOSPI_DCYC_POS) : 0)),
			h_to_le_32(OPI_CMD(SPIFLASH_READ_STATUS)),
		},
		{
			h_to_le_32(OCTOSPI_MODE | OCTOSPI_WRITE_MODE),
			h_to_le_32(IS_OCTOSPI ? OCTOSPI_CCR_WRITE_ENABLE : QSPI_CCR_WRITE_ENABLE),
			h_to_le_32(stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK),
			h_to_le_32(OPI_CMD(SPIFLASH_WRITE_ENABLE)),
		},
		{
			h_to_le_32(OCTOSPI_MODE | OCTOSPI_WRITE_MODE),
			h_to_le_32(IS_OCTOSPI ? OCTOSPI_CCR_PAGE_PROG : QSPI_CCR_PAGE_PROG),
			h_to_le_32(stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK),
			h_to_le_32(OPI_CMD(stmqspi_info->dev.pprog_cmd)),
		},
	};

	if (IS_OCTOSPI) {
		write_code = stmoctospi_write_code;
		write_code_size = sizeof(stmoctospi_write_code);
	} else {
		write_code = stmqspi_write_code;
		write_code_size = sizeof(stmqspi_write_code);
	}

	dual = (stmqspi_info->saved_cr & (1<<SPI_DUAL_FLASH)) ? 1 : 0;

	/* force reasonable defaults */
	fifosize = stmqspi_info->dev.sectorsize ?
		stmqspi_info->dev.sectorsize : stmqspi_info->dev.size_in_bytes;
	pagesize = stmqspi_info->dev.pagesize;
	if (pagesize == 0)
		pagesize = (fifosize <= 0x100) ? fifosize : 0x100;

	/* adjust sizes according to dual flash mode */
	pagesize = stmqspi_info->dev.pagesize << dual;
	fifosize = stmqspi_info->dev.sectorsize << dual;

	/* memory buffer, we assume sectorsize to be a power of 2 times pagesize */
	maxsize = target_get_working_area_avail(target);
	if (maxsize < write_code_size + 2 * sizeof(uint32_t) + pagesize) {
		LOG_WARNING("not enough working area, can't do QSPI page writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* fifo size at most sector size, and multiple of page size */
	maxsize -= (write_code_size + 2 * sizeof(uint32_t));
	fifosize = ((maxsize < fifosize) ? maxsize : fifosize) & ~(pagesize - 1);

	if (target_alloc_working_area_try(target,
		write_code_size + 2 * sizeof(uint32_t) + fifosize, &write_algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* prepare flash write code, excluding ccr_buffer */
	retval = target_write_buffer(target, write_algorithm->address,
		write_code_size - sizeof(ccr_buffer), write_code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare QSPI/OCTOSPI_CCR register values */
	retval = target_write_buffer(target, write_algorithm->address
		+ write_code_size - sizeof(ccr_buffer),
		sizeof(ccr_buffer), (uint8_t *) ccr_buffer);
	if (retval != ERROR_OK)
		goto err;

	/* target buffer starts right after flash_write_code, i. e.
	 * wp and rp are implicitly included in buffer!!! */
	fifo_start = write_algorithm->address + write_code_size + 2 * sizeof(uint32_t);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* count (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* pagesize */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);	/* offset into flash address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* QSPI/OCTOSPI io_base */
	init_reg_param(&reg_params[4], "r8", 32, PARAM_OUT);	/* fifo start */
	init_reg_param(&reg_params[5], "r9", 32, PARAM_OUT);	/* fifo end + 1 */

	buf_set_u32(reg_params[0].value, 0, 32, count);
	buf_set_u32(reg_params[1].value, 0, 32, pagesize);
	buf_set_u32(reg_params[2].value, 0, 32, offset);
	buf_set_u32(reg_params[3].value, 0, 32, io_base);
	buf_set_u32(reg_params[4].value, 0, 32, fifo_start);
	buf_set_u32(reg_params[5].value, 0, 32, fifo_start + fifosize);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* after breakpoint instruction (halfword) one nop (halfword) and
	 * ccr_buffer follow till end of code */
	exit_point = write_algorithm->address + write_code_size
	 - (sizeof(ccr_buffer) + sizeof(uint32_t));

	retval = target_run_flash_async_algorithm(target, buffer, count, 1,
			0, NULL,
			6, reg_params,
			write_algorithm->address + write_code_size,
			fifosize + 2 * sizeof(uint32_t),
			write_algorithm->address, exit_point,
			&armv7m_info);

	remaining = buf_get_u32(reg_params[0].value, 0, 32);
	if ((retval == ERROR_OK) && remaining)
		retval = ERROR_FLASH_OPERATION_FAILED;

	if (retval != ERROR_OK) {
		offset = buf_get_u32(reg_params[2].value, 0, 32);
		LOG_ERROR("flash write failed at address 0x%" PRIx32 ", remaining 0x%" PRIx32,
			offset, remaining);
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

err:
	target_free_working_area(target, write_algorithm);

	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int stmqspi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	int retval, sector, dual, octal_dtr;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Write beyond end of flash. Extra data discarded.");
		count = bank->size - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset < (bank->sectors[sector].offset + bank->sectors[sector].size))
			&& ((offset + count - 1) >= bank->sectors[sector].offset)
			&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	dual = (stmqspi_info->saved_cr & (1<<SPI_DUAL_FLASH)) ? 1 : 0;
	octal_dtr = IS_OCTOSPI && (stmqspi_info->saved_ccr & (1<<OCTOSPI_DDTR));
	if ((dual || octal_dtr) & ((offset & 1) != 0 || (count & 1) != 0)) {
		LOG_ERROR("In dual-QSPI and octal-DTR modes writes must be two byte aligned: "
			"%s: address=0x%08" PRIx32 " len=0x%08" PRIx32, __func__, offset, count);
		return ERROR_FAIL;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | (1<<SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return qspi_write_block(bank, buffer, offset, count);
}

/* Return ID of flash device(s) */
/* On exit, indirect mode is kept */
static int read_flash_id(struct flash_bank *bank, uint32_t *id1, uint32_t *id2)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	int shift, retval;
	uint8_t byte;

	if ((target->state != TARGET_HALTED) && (target->state != TARGET_RESET)) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | (1<<SPI_ABORT));
	if (retval != ERROR_OK)
		goto err;

	/* Poll WIP */
	retval = wait_till_ready(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Read three bytes per chip */
	retval = target_write_u32(target, io_base + SPI_DLR,
		(stmqspi_info->saved_cr & (1<<SPI_DUAL_FLASH) ? 6 : 3) - 1);
	if (retval != ERROR_OK)
		goto err;

	/* Read id */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_READ_MODE, OCTOSPI_CCR_READ_ID, SPIFLASH_READ_ID);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_READ_ID);
	if (retval != ERROR_OK)
		goto err;

	/* Dummy address 0, only required for 8-line mode */
	retval = target_write_u32(target, io_base + SPI_AR, 0);
	if (retval != ERROR_OK)
		goto err;

	/* Read ID from Data Register */
	for (shift = 0; shift <= 16; shift += 8) {
		if ((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) |
			(1<<SPI_FSEL_FLASH))) != (1<<SPI_FSEL_FLASH)) {
			retval = target_read_u8(target, io_base + SPI_DR, &byte);
			if (retval != ERROR_OK)
				goto err;
			*id1 |= ((uint32_t) byte) << shift;
		}
		if ((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) |
			(1<<SPI_FSEL_FLASH))) != 0) {
			retval = target_read_u8(target, io_base + SPI_DR, &byte);
			if (retval != ERROR_OK)
				goto err;
			*id2 |= ((uint32_t) byte) << shift;
		}
	}

	if ((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) |
		(1<<SPI_FSEL_FLASH))) != (1<<SPI_FSEL_FLASH))
		if ((*id1 == 0x000000) || (*id1 == 0xFFFFFF)) {
			/* no id retrieved, so id must be set manually */
			LOG_INFO("No id from flash1");
			retval = ERROR_TARGET_NOT_EXAMINED;
		}

	if ((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) |
		(1<<SPI_FSEL_FLASH))) != (0<<SPI_FSEL_FLASH))
		if ((*id2 == 0x000000) || (*id2 == 0xFFFFFF)) {
			/* no id retrieved, so id must be set manually */
			LOG_INFO("No id from flash2");
			retval = ERROR_TARGET_NOT_EXAMINED;
		}

err:
	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int stmqspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	uint32_t io_base = stmqspi_info->io_base;
	uint32_t id1 = 0, id2 = 0;
	const struct flash_device *p;
	int dual, fsize, retval;

	if (stmqspi_info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		if (bank->sectors)
			free(bank->sectors);
		bank->sectors = NULL;
		memset(&stmqspi_info->dev, 0, sizeof(stmqspi_info->dev));
		stmqspi_info->probed = 0;
	}

	stmqspi_info->octo = (READ_REG(OCTOSPI_MAGIC) == OCTO_MAGIC_ID);

	/* save current FSEL and DFM bits in QSPI/OCTOSPI_CR, current QSPI/OCTOSPI_CCR value */
	stmqspi_info->saved_cr = READ_REG(SPI_CR);
	if (retval == ERROR_OK)
		stmqspi_info->saved_ccr = READ_REG(SPI_CCR);

	if (IS_OCTOSPI) {
		uint32_t mtyp;

		mtyp = ((READ_REG(OCTOSPI_DCR1) & OCTOSPI_MTYP_MASK))>>OCTOSPI_MTYP_POS;
		if (retval == ERROR_OK)
			stmqspi_info->saved_tcr = READ_REG(OCTOSPI_TCR);
		if (retval == ERROR_OK)
			stmqspi_info->saved_ir = READ_REG(OCTOSPI_IR);
		if ((mtyp != 0x0) && (mtyp != 0x1)) {
			retval = ERROR_FAIL;
			LOG_ERROR("Only regular SPI protocol supported in OCTOSPI");
		}
		if (retval == ERROR_OK) {
			LOG_DEBUG("OCTOSPI at 0x%08" PRIx32 ", io_base at 0x%08" PRIx32 ", OCTOSPI_CR 0x%08"
				PRIx32 ", OCTOSPI_CCR 0x%08" PRIx32 ", %d-byte addr", bank->base, io_base,
				stmqspi_info->saved_cr, stmqspi_info->saved_ccr, SPI_ADSIZE);
		} else {
			LOG_ERROR("No OCTOSPI at io_base 0x%08" PRIx32, io_base);
			stmqspi_info->probed = 0;
			stmqspi_info->dev.name = "none";
			return ERROR_FAIL;
		}
	} else {
		/* check that QSPI is actually present at all */
		if (retval == ERROR_OK) {
			LOG_DEBUG("QSPI at 0x%08" PRIx32 ", io_base at 0x%08" PRIx32 ", QSPI_CR 0x%08"
				PRIx32 ", QSPI_CCR 0x%08" PRIx32 ", %d-byte addr", bank->base, io_base,
				stmqspi_info->saved_cr, stmqspi_info->saved_ccr, SPI_ADSIZE);
		} else {
			LOG_ERROR("No QSPI at io_base 0x%08" PRIx32, io_base);
			stmqspi_info->probed = 0;
			stmqspi_info->dev.name = "none";
			return ERROR_FAIL;
		}
	}

	/* read and decode flash ID; returns in memory mapped mode */
	retval = read_flash_id(bank, &id1, &id2);
	set_mm_mode(bank);
	LOG_DEBUG("id1 0x%06" PRIx32 ", id2 0x%06" PRIx32, id1, id2);
	if (retval == ERROR_TARGET_NOT_EXAMINED) {
		/* no id retrieved, so id must be set manually */
		LOG_INFO("No id - set flash parameters manually");
		return ERROR_OK;
	}

	if (retval != ERROR_OK)
		return retval;

	/* identify flash1 */
	for (p = flash_devices; id1 && p->name ; p++) {
		if (p->device_id == id1) {
			memcpy(&stmqspi_info->dev, p, sizeof(stmqspi_info->dev));
			LOG_INFO("flash1 \'%s\' id = 0x%06" PRIx32 " size = %lukbytes",
				 p->name, id1, p->size_in_bytes>>10);
			break;
		}
	}

	if (id1 && !p->name) {
		LOG_INFO("Unknown flash1 device id = 0x%06" PRIx32
				" - set flash parameters manually", id1);
		return ERROR_OK;
	}

	/* identify flash2 */
	for (p = flash_devices; id2 && p->name ; p++) {
		if (p->device_id == id2) {
			LOG_INFO("flash2 \'%s\' id = 0x%06" PRIx32 " size = %lukbytes",
				 p->name, id2, p->size_in_bytes>>10);

			if (!stmqspi_info->dev.name)
				memcpy(&stmqspi_info->dev, p, sizeof(stmqspi_info->dev));
			else
				if ((stmqspi_info->dev.erase_cmd != p->erase_cmd) ||
					(stmqspi_info->dev.chip_erase_cmd != p->chip_erase_cmd) ||
					(stmqspi_info->dev.pagesize != p->pagesize) ||
					(stmqspi_info->dev.sectorsize != p->sectorsize) ||
					(stmqspi_info->dev.size_in_bytes != p->size_in_bytes)) {
					LOG_WARNING("Incompatible flash1/flash2 devices");
				}
			break;
		}
	}

	if (id2 && !p->name) {
		LOG_INFO("Unknown flash2 device id = 0x%06" PRIx32
				" - set flash parameters manually", id2);
		return ERROR_OK;
	}

	/* Set correct size value */
	dual = (stmqspi_info->saved_cr & (1<<SPI_DUAL_FLASH)) ? 1 : 0;
	bank->size = stmqspi_info->dev.size_in_bytes << dual;

	fsize = ((READ_REG(SPI_DCR)>>SPI_FSIZE_POS) & ((1U<<SPI_FSIZE_LEN) - 1));
	LOG_DEBUG("FSIZE = 0x%04x", fsize);
	if (bank->size != (1U<<(fsize + 1)))
		LOG_WARNING("FSIZE field in QSPI_DCR(1) doesn't match actual capacity.");

	/* if no sectors, then treat whole flash as single sector */
	if (stmqspi_info->dev.sectorsize == 0)
		stmqspi_info->dev.sectorsize = stmqspi_info->dev.size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = stmqspi_info->dev.size_in_bytes / stmqspi_info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * (stmqspi_info->dev.sectorsize << dual);
		sectors[sector].size = (stmqspi_info->dev.sectorsize << dual);
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	stmqspi_info->probed = 1;

	return ERROR_OK;
}

static int stmqspi_auto_probe(struct flash_bank *bank)
{
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;

	if (stmqspi_info->probed)
		return ERROR_OK;
	return stmqspi_probe(bank);
}

static int stmqspi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_stmqspi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;

	if (!(stmqspi_info->probed)) {
		snprintf(buf, buf_size,
			"\nQSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "flash%s%s \'%s\', device id = 0x%06" PRIx32
			", flash size = %dkBytes\n(page size = %d, read = 0x%02" PRIx8
			", fread = 0x%02" PRIx8 ", pprog = 0x%02" PRIx8
			", mass_erase = 0x%02" PRIx8 ", sector size = %ldkBytes"
			", sector_erase = 0x%02" PRIx8 ")",
			((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) |
			(1<<SPI_FSEL_FLASH))) != (1<<SPI_FSEL_FLASH)) ? "1" : "",
			((stmqspi_info->saved_cr & ((1<<SPI_DUAL_FLASH) |
			(1<<SPI_FSEL_FLASH))) != (0<<SPI_FSEL_FLASH)) ? "2" : "",
			stmqspi_info->dev.name, stmqspi_info->dev.device_id, bank->size>>10,
			stmqspi_info->dev.pagesize, stmqspi_info->dev.read_cmd,
			stmqspi_info->dev.fread_cmd, stmqspi_info->dev.pprog_cmd,
			stmqspi_info->dev.chip_erase_cmd, stmqspi_info->dev.sectorsize>>10,
			stmqspi_info->dev.erase_cmd);

	return ERROR_OK;
}

static const struct command_registration stmqspi_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = stmqspi_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	{
		.name = "set",
		.handler = stmqspi_handle_set,
		.mode = COMMAND_EXEC,
		.usage = "bank_id name chip_size page_size read_cmd pprg_cmd "
			"[ mass_erase_cmd ] [ sector_size sector_erase_cmd ]",
		.help = "Set params of single flash chip",
	},
	{
		.name = "cmd",
		.handler = stmqspi_handle_cmd,
		.mode = COMMAND_EXEC,
		.usage = "bank_id num_resp cmd_byte ...",
		.help = "Send low-level command cmd_byte and following bytes or read num_resp.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stmqspi_command_handlers[] = {
	{
		.name = "stmqspi",
		.mode = COMMAND_ANY,
		.help = "stmqspi flash command group",
		.usage = "",
		.chain = stmqspi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stmqspi_flash = {
	.name = "stmqspi",
	.commands = stmqspi_command_handlers,
	.flash_bank_command = stmqspi_flash_bank_command,
	.erase = stmqspi_erase,
	.protect = stmqspi_protect,
	.write = stmqspi_write,
	.read = default_flash_read,
	.probe = stmqspi_probe,
	.auto_probe = stmqspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stmqspi_protect_check,
	.info = get_stmqspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

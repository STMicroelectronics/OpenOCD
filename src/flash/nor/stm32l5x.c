/***************************************************************************
 *   Copyright (C) 2019 by STMicroelectronics                              *
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
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* Regarding performance:
 *
 * The reference manual for STM32L5 is RM0438
 */

/* Erase time can be as high as 1000ms, 10x this and it's toast... */
#define FLASH_ERASE_TIMEOUT 10000
#define FLASH_WRITE_TIMEOUT 5

#define FLASH_ACR		0x00
#define FLASH_KEYR		0x08
#define FLASH_SECKEYR	0x0C
#define FLASH_OPTKEYR	0x10
#define FLASH_SR		0x20
#define FLASH_SECSR		0x24
#define FLASH_CR		0x28
#define FLASH_SECCR		0x2C
#define FLASH_OPTR      0x40
#define FLASH_WRP1AR	0x58
#define FLASH_WRP1BR	0x5C
#define FLASH_WRP2AR	0x68
#define FLASH_WRP2BR	0x6C
#define FLASH_SECBB1R1	0x80
#define FLASH_SECBB1R2	0x84
#define FLASH_SECBB1R3	0x88
#define FLASH_SECBB1R4	0x8C
#define FLASH_SECBB2R1	0xA0
#define FLASH_SECBB2R2	0xA4
#define FLASH_SECBB2R3	0xA8
#define FLASH_SECBB2R4	0xAC

/* FLASH_ACR bits */
#define FLASH_ACR__LATENCY		(1<<0)
#define FLASH_ACR__RUN_PD		(1<<13)
#define FLASH_ACR__SLEEP_PD		(1<<14)

/* FLASH_CR register bits */
#define FLASH_PG         (1 << 0)
#define FLASH_PER        (1 << 1)
#define FLASH_MER1       (1 << 2)
#define FLASH_PNB        (1 << 3)
#define FLASH_BKER       (1 << 11) /* For Dual Bank devices */
#define FLASH_MER2       (1 << 15) /* For Dual Bank devices */
#define FLASH_START      (1 << 16)
#define FLASH_OPTSTRT    (1 << 17)
#define FLASH_EOPIE      (1 << 24)
#define FLASH_ERRIE      (1 << 25)
#define FLASH_RDERRIE    (1 << 26)
#define FLASH_OBL_LAUNCH (1 << 27)
#define FLASH_OPTLOCK    (1 << 30)
#define FLASH_LOCK       (1 << 31)

/* The sector number encoding */
#define FLASH_SNB(a)	(a << 3)

/* FLASH_SR register bits */
#define FLASH_BSY      (1 << 16) /* Operation in progress */
#define FLASH_OPTWERR  (1 << 13) /* Option modification error */
#define FLASH_PGSERR   (1 << 7)  /* Programming sequence error */
#define FLASH_SIZERR   (1 << 6)  /* Programming size error */
#define FLASH_PGAERR   (1 << 5)  /* Programming alignment error */
#define FLASH_WRPERR   (1 << 4)  /* Write protection error */
#define FLASH_PROGERR  (1 << 3)  /* Write protection error */
#define FLASH_OPERR    (1 << 1)  /* Operation error */
#define FLASH_EOP      (1 << 0)  /* End of operation */

#define FLASH_ERROR (FLASH_OPTWERR | FLASH_PGSERR | FLASH_SIZERR | FLASH_PGAERR \
			| FLASH_WRPERR | FLASH_PROGERR | FLASH_OPERR)

/* register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

/* option bytes */
#define TZEN       (1 << 31)
#define DBANK      (1 << 22)	/* dual flash bank only */
#define DUALBANK   (1 << 21)	/* dual flash bank only */
#define WWWG_SW    (1 << 19)
#define IWDG_STDBY (1 << 18)
#define IWDG_STOP  (1 << 17)
#define IDWG_SW    (1 << 16)

#define ALL_FF         0xFFFFFFFF

#define DBGMCU_IDCODE_REGISTER	0xE0044000
#define SCS_SAU_REGISTER	0xE000EDD0
#define SAU_ALL_NS		0x02


struct stm32l5x_rev {
	uint16_t rev;
	const char *str;
};

struct stm32x_options {
	uint32_t user_options;
	uint8_t RDP;
	uint8_t window_watchdog_selection;
	uint8_t independent_watchdog_standby;
	uint8_t independent_watchdog_stop;
	uint8_t independent_watchdog_selection;
	/* Two zone of wpr by bank */
	uint8_t wpr1a_start;
	uint8_t wpr1a_end;
	uint8_t wpr1b_start;
	uint8_t wpr1b_end;
	uint8_t wpr2a_start;
	uint8_t wpr2a_end;
	uint8_t wpr2b_start;
	uint8_t wpr2b_end;
    /* Fixme: Handle PCROP */
};

struct stm32l5x_part_info {
	uint16_t id;
	const char *device_str;
	const struct stm32l5x_rev *revs;
	size_t num_revs;
	unsigned int page_size;
	uint16_t max_flash_size_kb;
	uint8_t has_dual_bank;
	uint16_t first_bank_sectors; /* used to convert sector number */
	uint16_t hole_sectors;       /* use to recalculate the real sector number */
	uint32_t flash_base;         /* Flash controller registers location */
	uint32_t fsize_base;         /* Location of FSIZE register */
};

struct stm32l5x_flash_bank {
	int probed;
	uint32_t idcode;
	uint32_t user_bank_size;
	uint32_t flash_base;    /* Address of flash memory */
	struct stm32x_options option_bytes;
	struct stm32l5x_part_info *part_info;
	bool tz_enabled;
};

static const struct stm32l5x_rev stm32_472_revs[] = {
	{ 0x1000, "A" },
};

static struct stm32l5x_part_info stm32l5x_parts[] = {
	{
	  .id					= 0x472,
	  .revs					= stm32_472_revs,
	  .num_revs				= ARRAY_SIZE(stm32_472_revs),
	  .device_str			= "STM32L55/L56xx",	/* 512K */
	  .page_size			= 2048,
	  .max_flash_size_kb	= 512,
	  .has_dual_bank		= 1,
	  .first_bank_sectors	= 128,
	  .hole_sectors			= 0,
	  .flash_base			= 0x40022000,
	  .fsize_base			= 0x0BFA05E0,
	},
};

static int stm32x_unlock_reg(struct flash_bank *bank);
static int stm32x_probe(struct flash_bank *bank);

FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32l5x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32x_info = malloc(sizeof(struct stm32l5x_flash_bank));
	bank->driver_priv = stm32x_info;

	stm32x_info->probed = 0;
	stm32x_info->tz_enabled = false;
	stm32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static int stm32x_set_sec_bb_reg(struct flash_bank *bank) {
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	int retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R1, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R2, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R3, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R4, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R1, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R2, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R3, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R4, ALL_FF);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_reset_sec_bb_reg(struct flash_bank *bank) {
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	int retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R1, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R2, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R3, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB1R4, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R1, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R2, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R3, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_SECBB2R4, 0);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	return target_read_u32(bank->target, stm32x_info->flash_base + FLASH_SR, status);
}

static int stm32x_wait_status_busy(struct flash_bank *bank, uint32_t *status, int timeout)
{
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32x_get_flash_status(bank, status);
		if (retval != ERROR_OK) {
			LOG_INFO("wait_status_busy, target_*_u32 : error : remote address 0x%x", stm32x_info->flash_base);
			return retval;
		}

		if ((*status & FLASH_BSY) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_INFO("wait_status_busy, time out expired");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (*status & FLASH_ERROR) {
		if (*status & FLASH_WRPERR)
			LOG_ERROR("wait_status_busy, WRPERR : error : remote address 0x%x", stm32x_info->flash_base);
		else
			LOG_ERROR("wait_status_busy, flash errors occurred");

		retval = ERROR_FAIL;
	}

	/* Clear all FLASH_SR flags */
	if (*status & (FLASH_ERROR | FLASH_EOP)) {
		/* If this operation fails, we ignore it and report the original retval */
		target_write_u32(target, stm32x_info->flash_base + FLASH_SR,
					*status & (FLASH_ERROR | FLASH_EOP));
	}

	return retval;
}

static int stm32x_unlock_reg(struct flash_bank *bank)
{
	uint32_t ctrl;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;

	/* first check if not already unlocked
	 * otherwise writing on STM32_FLASH_KEYR will fail
	 */
	int retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_LOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32x_unlock_option_reg(struct flash_bank *bank)
{
	uint32_t ctrl;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;

	int retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_OPTLOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_OPTLOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_OPTCR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32x_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;

	/* read current option bytes */
	int retval = target_read_u32(target, stm32x_info->flash_base + FLASH_OPTR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	stm32x_info->option_bytes.user_options = optiondata >> 8;
	stm32x_info->option_bytes.RDP = optiondata & 0xff;

	if (optiondata & WWWG_SW)
		stm32x_info->option_bytes.window_watchdog_selection = 1;
	else
		stm32x_info->option_bytes.window_watchdog_selection = 0;

	if (optiondata & IWDG_STDBY)
		stm32x_info->option_bytes.independent_watchdog_standby = 1;
	else
		stm32x_info->option_bytes.independent_watchdog_standby = 0;

	if (optiondata & IWDG_STOP)
		stm32x_info->option_bytes.independent_watchdog_stop = 1;
	else
		stm32x_info->option_bytes.independent_watchdog_stop = 0;

	if (optiondata & IDWG_SW)
		stm32x_info->option_bytes.independent_watchdog_selection = 1;
	else
		stm32x_info->option_bytes.independent_watchdog_selection = 0;

	/* Read wrp options */
	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_WRP1AR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32x_info->option_bytes.wpr1a_start =  optiondata         & 0xff;
	stm32x_info->option_bytes.wpr1a_end   = (optiondata >> 16)  & 0xff;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_WRP1BR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32x_info->option_bytes.wpr1b_start =  optiondata         & 0xff;
	stm32x_info->option_bytes.wpr1b_end   = (optiondata >> 16)  & 0xff;

	/* for double bank devices */
	if (stm32x_info->part_info->has_dual_bank) {
		retval = target_read_u32(target, stm32x_info->flash_base + FLASH_WRP2AR, &optiondata);
		if (retval != ERROR_OK)
			return retval;
		stm32x_info->option_bytes.wpr2a_start =  optiondata         & 0xff;
		stm32x_info->option_bytes.wpr2a_end   = (optiondata >> 16)  & 0xff;

		retval = target_read_u32(target, stm32x_info->flash_base + FLASH_WRP2BR, &optiondata);
		if (retval != ERROR_OK)
			return retval;
		stm32x_info->option_bytes.wpr2b_start =  optiondata         & 0xff;
		stm32x_info->option_bytes.wpr2b_end   = (optiondata >> 16)  & 0xff;
	}

	/* fixme read PCRop options */

	if (stm32x_info->option_bytes.RDP != 0xAA)
		LOG_INFO("Device RDP is set");

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t optiondata;

	int retval = stm32x_unlock_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_unlock_option_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	optiondata = (stm32x_info->option_bytes.user_options << 8);
	optiondata |= stm32x_info->option_bytes.RDP;

	if (stm32x_info->option_bytes.window_watchdog_selection)
		optiondata |= WWWG_SW;
	else
		optiondata &= ~WWWG_SW;

	if (stm32x_info->option_bytes.independent_watchdog_standby)
		optiondata |= IWDG_STDBY;
	else
		optiondata &= ~IWDG_STDBY;

	if (stm32x_info->option_bytes.independent_watchdog_stop)
		optiondata |= IWDG_STOP;
	else
		optiondata &= ~IWDG_STOP;

	if (stm32x_info->option_bytes.independent_watchdog_selection)
		optiondata |= IDWG_SW;
	else
		optiondata &= ~IDWG_SW;

	/* write options registers */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_OPTR, optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* write wrp options */
	optiondata = (stm32x_info->option_bytes.wpr1a_end << 16) | stm32x_info->option_bytes.wpr1a_start;
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_WRP1AR, optiondata);
	if (retval != ERROR_OK)
		return retval;

	optiondata = (stm32x_info->option_bytes.wpr1b_end << 16) | stm32x_info->option_bytes.wpr1b_start;
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_WRP1BR, optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* write wrp options for double bank devices */
	if (stm32x_info->part_info->has_dual_bank) {
		optiondata = (stm32x_info->option_bytes.wpr2a_end << 16) | stm32x_info->option_bytes.wpr2a_start;
		retval = target_write_u32(target, stm32x_info->flash_base + FLASH_WRP2AR, optiondata);
		if (retval != ERROR_OK)
			return retval;

		optiondata = (stm32x_info->option_bytes.wpr2b_end << 16) | stm32x_info->option_bytes.wpr2b_start;
		retval = target_write_u32(target, stm32x_info->flash_base + FLASH_WRP2BR, optiondata);
		if (retval != ERROR_OK)
			return retval;
	}

	/* fixme: Add PCROPxx registers write */

	/* start options programming cycle */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_OPTSTRT);
	if (retval != ERROR_OK)
		return retval;

	/* wait for completion */
	uint32_t flash_sr;
	retval = stm32x_wait_status_busy(bank, &flash_sr, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	if (flash_sr & FLASH_ERROR) {
		LOG_ERROR("flash operation failed");
		return FLASH_ERROR;
	}

	/* FIXME need to lock flash and options after failure too */

	/* relock option register */
	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_OPTLOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	const struct stm32l5x_part_info *l5_part_info = stm32x_info->part_info;

	/* read 'write protection' settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = 0; i < bank->num_sectors; i++) {
		if (i < l5_part_info->first_bank_sectors) {
			if ((i >= stm32x_info->option_bytes.wpr1a_start &&
				 i <= stm32x_info->option_bytes.wpr1a_end) ||
				(i >= stm32x_info->option_bytes.wpr1b_start &&
				 i <= stm32x_info->option_bytes.wpr1b_end))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		}
		else {
			if (((i - l5_part_info->first_bank_sectors) >= stm32x_info->option_bytes.wpr2a_start &&
				 (i - l5_part_info->first_bank_sectors) <= stm32x_info->option_bytes.wpr2a_end) ||
				((i - l5_part_info->first_bank_sectors) >= stm32x_info->option_bytes.wpr2b_start &&
				 (i - l5_part_info->first_bank_sectors) <= stm32x_info->option_bytes.wpr2b_end))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		}
	}

	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	const struct stm32l5x_part_info *l5_part_info = stm32x_info->part_info;
	unsigned int i;
	int retval;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	uint32_t flash_base_orig = stm32x_info->flash_base;

	stm32x_read_options(bank);

	if (stm32x_info->tz_enabled && stm32x_info->option_bytes.RDP == 0xAA) {
		retval = stm32x_set_sec_bb_reg(bank);
		if (retval != ERROR_OK)
			return retval;

		stm32x_info->flash_base += 4;
	}

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register
	2. Set the PER bit and select the sector you wish to erase (SNB)
	   in the FLASH_CR register
	   if there is the second bank, set the FLASH_BKER bank erase bit
	3. Set the FLASH_START bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	*/

	for (i = first; i <= (unsigned int)last; i++) {
		if (i < l5_part_info->first_bank_sectors)
			retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR,
					FLASH_PER | FLASH_SNB(i) | FLASH_START);
		else
			retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR,
					FLASH_BKER | FLASH_PER
					| FLASH_SNB((i + l5_part_info->hole_sectors)) | FLASH_START);
		if (retval != ERROR_OK) {
			stm32x_info->flash_base = flash_base_orig;
			LOG_ERROR("erase sector error %d", i);
			return retval;
		}

		uint32_t flash_sr;
		retval = stm32x_wait_status_busy(bank, &flash_sr, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK) {
			stm32x_info->flash_base = flash_base_orig;
			LOG_ERROR("erase time-out error sector %d", i);
			return retval;
		}

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		LOG_ERROR("error during the lock of flash");
		return retval;
	}

	if (stm32x_info->tz_enabled && stm32x_info->option_bytes.RDP == 0xAA) {
		stm32x_info->flash_base = flash_base_orig;
		retval = stm32x_reset_sec_bb_reg(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	const struct stm32l5x_part_info *l5_part_info = stm32x_info->part_info;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = first; i <= last; i++) {
		if (set)
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}

	/* analyse the sectors protected to create zone of wpr */
	/* zone in first bank only */
	if ((first < l5_part_info->first_bank_sectors) && (last < l5_part_info->first_bank_sectors)) {
		if (set) {
			stm32x_info->option_bytes.wpr1a_start = first;
			stm32x_info->option_bytes.wpr1a_end = last;
		} else {
			/* Fixme, should check old value */
			/* if (stm32x_info->option_bytes.wpr1a_start ) */
			stm32x_info->option_bytes.wpr1a_start = 0xff;
			stm32x_info->option_bytes.wpr1a_end = 0;
		}
		stm32x_info->option_bytes.wpr1b_start = 0xff;
		stm32x_info->option_bytes.wpr1b_end = 0;
	}
	/* zone in second bank only */
	else if (first >= l5_part_info->first_bank_sectors) {
		if (set) {
			stm32x_info->option_bytes.wpr2a_start = first - l5_part_info->first_bank_sectors;
			stm32x_info->option_bytes.wpr2a_end = last - l5_part_info->first_bank_sectors;
		} else {
			/* Fixme, should check old value */
			/* if (stm32x_info->option_bytes.wpr2a_start ) */
			stm32x_info->option_bytes.wpr2a_start = 0xff;
			stm32x_info->option_bytes.wpr2a_end = 0;
		}
		stm32x_info->option_bytes.wpr2b_start = 0xff;
		stm32x_info->option_bytes.wpr2b_end = 0;
	}
	/* zone spread over the two banks */
	else if ((first < l5_part_info->first_bank_sectors) && (last >= l5_part_info->first_bank_sectors)) {
		if (set) {
			stm32x_info->option_bytes.wpr1a_start = first;
			stm32x_info->option_bytes.wpr1a_end = l5_part_info->first_bank_sectors-1;
			stm32x_info->option_bytes.wpr2a_start = 0;
			stm32x_info->option_bytes.wpr2a_end = last - l5_part_info->first_bank_sectors;
		} else {
			/* Fixme, should check old value */
			/* if (stm32x_info->option_bytes.wpr1a_start ) */
			stm32x_info->option_bytes.wpr1a_start = 0xff;
			stm32x_info->option_bytes.wpr1a_end = 0;
			stm32x_info->option_bytes.wpr2a_start = 0xff;
			stm32x_info->option_bytes.wpr2a_end = 0;
		}
		stm32x_info->option_bytes.wpr1b_start = 0xff;
		stm32x_info->option_bytes.wpr1b_end = 0;
		stm32x_info->option_bytes.wpr2b_start = 0xff;
		stm32x_info->option_bytes.wpr2b_end = 0;
	}

	retval = stm32x_write_options(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
				uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	int retval = ERROR_OK;

	static const uint8_t stm32l5_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32l5x.inc"
	};

	if (target_alloc_working_area(target, sizeof(stm32l5_flash_write_code),
					&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
					sizeof(stm32l5_flash_write_code),
					stm32l5_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address  */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count (word-64bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);		/* flash_reg addr */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, stm32x_info->flash_base);

	retval = target_run_flash_async_algorithm(target,
						buffer,
						count,
						8, /* Size of block in bytes */
						0, NULL,
						5, reg_params,
						source->address,
						source->size,
						write_algorithm->address, 0,
						&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_INFO("error executing stm32l5x flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR;

		if (error & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, stm32x_info->flash_base + FLASH_SR, error);
			retval = ERROR_FAIL;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int stm32x_write_block_without_loader(struct flash_bank *bank, const uint8_t *buffer,
				uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t address = bank->base + offset;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	int retval = ERROR_OK;

	uint32_t flash_cr, flash_sr;

	/* set PG in FLASH_CR */
	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &flash_cr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, flash_cr | FLASH_PG);
	if (retval != ERROR_OK)
		return retval;


	/* write directly to flash memory */
	const uint8_t *src = buffer;
	while (count--) {
		retval = target_write_memory(target, address, 4, 2, src);
		if (retval != ERROR_OK)
			return retval;

		/* wait for BSY bit */
		stm32x_wait_status_busy(bank, &flash_sr, FLASH_WRITE_TIMEOUT);

		/* check for errors */
		if (flash_sr & FLASH_ERROR) {
			LOG_ERROR("flash operation failed");
			return ERROR_FAIL;
		}

		src += 8;
		address += 8;
	}

	/* reset PG in FLASH_CR */
	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &flash_cr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, flash_cr & ~FLASH_PG);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x7) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 8-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x7) {
		LOG_WARNING("Padding %d bytes to keep 8-byte write size", count & 7);
		count = (count + 7) & ~7;
		/* This pads the write chunk with random bytes by overrunning the
		 * write buffer. Padding with the erased pattern 0xff is purely
		 * cosmetic, as 8-byte flash words are ECC secured and the first
		 * write will program the ECC bits. A second write would need
		 * to reprogram these ECC bits.
		 * But this can only be done after erase!
		 */
	}

	uint32_t flash_base_orig = stm32x_info->flash_base;
	stm32x_read_options(bank);


	if (stm32x_info->tz_enabled && stm32x_info->option_bytes.RDP == 0xAA) {
		retval = stm32x_set_sec_bb_reg(bank);
		if (retval != ERROR_OK)
			return retval;

		stm32x_info->flash_base += 4;
	}

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	/* multiple words (8-byte block) to be programmed */
	if (stm32x_info->tz_enabled && stm32x_info->option_bytes.RDP == 0x55) {
		LOG_INFO("RDP is level 0.5, using direct flash programming since the specified work-area could be secure !");
		retval = stm32x_write_block_without_loader(bank, buffer, offset, count/8);
	} else
		retval = stm32x_write_block(bank, buffer, offset, count/8);

	if ((retval != ERROR_OK) && (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)) {
		stm32x_info->flash_base = flash_base_orig;
		LOG_INFO("block write failed");
		return retval;
	}

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	if (stm32x_info->tz_enabled && stm32x_info->option_bytes.RDP == 0xAA) {
		stm32x_info->flash_base = flash_base_orig;
		retval = stm32x_reset_sec_bb_reg(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int stm32x_read_id_code(struct flash_bank *bank, uint32_t *id)
{
	/* read stm32 device id register */
	int retval = target_read_u32(bank->target, DBGMCU_IDCODE_REGISTER, id);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	uint32_t i;
	uint16_t flash_size_in_kb;
	uint32_t device_id;
	uint32_t options;

	stm32x_info->probed = 0;

	int retval = stm32x_read_id_code(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;

	stm32x_info->idcode = device_id;

	LOG_INFO("Device id = 0x%08" PRIx32 "", device_id);

	for (unsigned int n = 0; n < ARRAY_SIZE(stm32l5x_parts); n++) {
		if ((device_id & 0xfff) == stm32l5x_parts[n].id)
			stm32x_info->part_info = &stm32l5x_parts[n];
	}

	if (!stm32x_info->part_info) {
		LOG_WARNING("Cannot identify target as a STM32L5xx family.");
		return ERROR_FAIL;
	}

	stm32x_info->flash_base = stm32x_info->part_info->flash_base;

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_OPTR, &options);
	if ((retval == ERROR_OK) && (options & TZEN)) {
		stm32x_info->tz_enabled = true;
		LOG_INFO("TZEN = 1 : Device secure, Trust Zone enabled by option bytes");
	}

	/* if device is Secure, we need to set SAU to all NonSecure to read flash_size register */
	if (stm32x_info->tz_enabled) {
		retval = target_write_u32(target, SCS_SAU_REGISTER, SAU_ALL_NS);
		if (retval != ERROR_OK)
			return retval;
	}
	/* get flash size from target */
	retval = target_read_u16(target, stm32x_info->part_info->fsize_base, &flash_size_in_kb);
	if (retval != ERROR_OK || flash_size_in_kb == 0 || flash_size_in_kb > stm32x_info->part_info->max_flash_size_kb) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
			stm32x_info->part_info->max_flash_size_kb);
		flash_size_in_kb = stm32x_info->part_info->max_flash_size_kb;
	}
	/* restore SCS_SAU_REGISTER reset value */
	if (stm32x_info->tz_enabled) {
		retval = target_write_u32(target, SCS_SAU_REGISTER, 0);
		if (retval != ERROR_OK)
			return retval;
	}

	if (stm32x_info->part_info->has_dual_bank) {
		/* get options for DUAL BANK */
		retval = target_read_u32(target, stm32x_info->flash_base + FLASH_OPTR, &options);
		/* test DBANK option (Default Dual bank page_size = 2048) */
		if ((options & DBANK) == 0) {
			stm32x_info->part_info->page_size = 4096; /* Single bank */
			LOG_INFO("Single bank flash activated by option bytes");
		}
		/* test if dual bank on a smaller device (hole between banks for sector erase) */
		else if ((options & DUALBANK) && (flash_size_in_kb < stm32x_info->part_info->max_flash_size_kb)) {
			stm32x_info->part_info->first_bank_sectors = \
						((flash_size_in_kb * 1024) / stm32x_info->part_info->page_size)/2;
			stm32x_info->part_info->hole_sectors = \
			            (((stm32x_info->part_info->max_flash_size_kb * 1024) / stm32x_info->part_info->page_size) /2) \
 			            - stm32x_info->part_info->first_bank_sectors;
		}
	}

	LOG_INFO("STM32L5xx flash size is %dkb, base address is " TARGET_ADDR_FMT, flash_size_in_kb, bank->base);

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (stm32x_info->user_bank_size) {
		flash_size_in_kb = stm32x_info->user_bank_size / 1024;
		LOG_INFO("ignoring flash probed value, using configured bank size: %d kbytes", flash_size_in_kb);
	}

	/* calculate numbers of sectors */
	uint32_t num_sectors =  (flash_size_in_kb * 1024) / stm32x_info->part_info->page_size;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->size = flash_size_in_kb * 1024;
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (bank->sectors == NULL) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	for (i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * stm32x_info->part_info->page_size;
		bank->sectors[i].size = stm32x_info->part_info->page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32x_info->probed = 1;
	return ERROR_OK;
}

static int stm32x_auto_probe(struct flash_bank *bank)
{
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;

	if (stm32x_info->probed)
		return ERROR_OK;

	return stm32x_probe(bank);
}

/* This method must return a string displaying information about the bank */
static int get_stm32x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;

	if (!stm32x_info->probed) {
		int retval = stm32x_probe(bank);
		if (retval != ERROR_OK) {
			snprintf(buf, buf_size, "Unable to find bank information.");
			return retval;
		}
	}

	const struct stm32l5x_part_info *info = stm32x_info->part_info;

	if (info) {
		const char *rev_str = NULL;
		uint16_t rev_id = stm32x_info->idcode >> 16;
		for (unsigned int i = 0; i < info->num_revs; i++) {
			if (rev_id == info->revs[i].rev) {
				rev_str = info->revs[i].str;

				if (rev_str != NULL) {
					snprintf(buf, buf_size, "%s - Rev: %s",
							stm32x_info->part_info->device_str, rev_str);
					return ERROR_OK;
				}
			}
		}

		snprintf(buf, buf_size, "%s - Rev: unknown (0x%04x)",
				stm32x_info->part_info->device_str, rev_id);
		return ERROR_OK;
	} else {
		snprintf(buf, buf_size, "Cannot identify target as a STM32L5x");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

/* Static method for mass erase stuff implementation */
static int stm32x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l5x_flash_bank *stm32x_info = bank->driver_priv;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t flash_base_orig = stm32x_info->flash_base;
	stm32x_read_options(bank);

	if (stm32x_info->tz_enabled && stm32x_info->option_bytes.RDP == 0xAA) {
		retval = stm32x_set_sec_bb_reg(bank);
		if (retval != ERROR_OK)
			return retval;

		stm32x_info->flash_base += 4;
	}

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	uint32_t flash_sr;
	retval = stm32x_wait_status_busy(bank, &flash_sr, FLASH_ERASE_TIMEOUT/10);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	uint32_t reg32;
	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	/* mass erase flash memory : if two banks, two bit to set: MER1 & MER2 */
	if (stm32x_info->part_info->has_dual_bank)
		reg32 |= FLASH_MER2;

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, reg32 | FLASH_MER1);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, reg32 | FLASH_MER1 | FLASH_START);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	retval = stm32x_wait_status_busy(bank, &flash_sr, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	retval = target_read_u32(target, stm32x_info->flash_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	retval = target_write_u32(target, stm32x_info->flash_base + FLASH_CR, reg32 | FLASH_LOCK);
	if (retval != ERROR_OK) {
		stm32x_info->flash_base = flash_base_orig;
		return retval;
	}

	if (stm32x_info->tz_enabled && stm32x_info->option_bytes.RDP == 0xAA) {
		stm32x_info->flash_base = flash_base_orig;
		retval = stm32x_reset_sec_bb_reg(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32l5x_flash_bank *stm32l5x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32l5x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* set readout protection */
	stm32l5x_info->option_bytes.RDP = 0;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to lock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD, "%s locked", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
{
	struct target *target = NULL;
	struct stm32l5x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	stm32x_info->option_bytes.RDP = 0xAA;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD, "%s unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	if (CMD_ARGC < 1) {
		command_print(CMD, "stm32l5x mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "stm32l5x mass erase complete");
	} else {
		command_print(CMD, "stm32l5x mass erase failed");
	}

	return retval;
}

COMMAND_HANDLER(stm32x_window_watchdog_selection)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "stm32l5x window_watchdog_soft_selection bank_id [enable|disable]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct stm32l5x_flash_bank *stm32x_info =  bank->driver_priv;

	retval = stm32x_unlock_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_unlock_option_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	bool enable = false;
	if (CMD_ARGC == 2)
		COMMAND_PARSE_ENABLE(CMD_ARGV[1], enable);

	if (enable)
		stm32x_info->option_bytes.window_watchdog_selection = 1;
	else
		stm32x_info->option_bytes.window_watchdog_selection = 0;

	 return stm32x_write_options(bank);
}

COMMAND_HANDLER(stm32x_watchdog_standby)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "stm32l5x independent_watchdog_standby bank_id [enable|disable]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct stm32l5x_flash_bank *stm32x_info =  bank->driver_priv;

	retval = stm32x_unlock_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_unlock_option_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	bool enable = false;
	if (CMD_ARGC == 2)
		COMMAND_PARSE_ENABLE(CMD_ARGV[1], enable);

	if (enable)
		stm32x_info->option_bytes.independent_watchdog_standby = 1;
	else
		stm32x_info->option_bytes.independent_watchdog_standby = 0;

	return stm32x_write_options(bank);
}

COMMAND_HANDLER(stm32x_watchdog_stop)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "stm32l5x independent_watchdog_stop bank_id [enable|disable]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct stm32l5x_flash_bank *stm32x_info =  bank->driver_priv;

	retval = stm32x_unlock_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_unlock_option_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	bool enable = false;
	if (CMD_ARGC == 2)
		COMMAND_PARSE_ENABLE(CMD_ARGV[1], enable);

	if (enable)
		stm32x_info->option_bytes.independent_watchdog_stop = 1;
	else
		stm32x_info->option_bytes.independent_watchdog_stop = 0;

	return stm32x_write_options(bank);
}

COMMAND_HANDLER(stm32x_watchdog_selection)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "stm32l5x independent_watchdog_soft_selection bank_id [enable|disable]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct stm32l5x_flash_bank *stm32x_info =  bank->driver_priv;

	retval = stm32x_unlock_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_unlock_option_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	bool enable = false;
	if (CMD_ARGC == 2)
		COMMAND_PARSE_ENABLE(CMD_ARGV[1], enable);

	if (enable)
		stm32x_info->option_bytes.independent_watchdog_selection = 1;
	else
		stm32x_info->option_bytes.independent_watchdog_selection = 0;

	 return stm32x_write_options(bank);
}


static const struct command_registration stm32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "window_watchdog_soft_selection",
		.handler = stm32x_window_watchdog_selection,
		.mode = COMMAND_EXEC,
		.usage = "window_watchdog_soft_selection bank_id ['enable'|'disable']",
		.help = "Software window watchdog selection.",
	},
	{
		.name = "independent_watchdog_standby",
		.handler = stm32x_watchdog_standby,
		.mode = COMMAND_EXEC,
		.usage = "independent_watchdog_standby bank_id ['enable'|'disable']",
		.help = "Freeze the independent watchdog counter in Standby mode.",
	},
	{
		.name = "independent_watchdog_stop",
		.handler = stm32x_watchdog_stop,
		.mode = COMMAND_EXEC,
		.usage = "independent_watchdog_stop bank_id ['enable'|'disable']",
		.help = "Freeze the independent watchdog counter in Stop mode.",
	},
	{
		.name = "independent_watchdog_soft_selection",
		.handler = stm32x_watchdog_selection,
		.mode = COMMAND_EXEC,
		.usage = "independent_watchdog_soft_selection bank_id ['enable'|'disable']",
		.help = "Software independent watchdog selection.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32l5x",
		.mode = COMMAND_ANY,
		.help = "stm32l5x flash command group",
		.usage = "",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32l5x_flash = {
	.name = "stm32l5x",
	.commands = stm32x_command_handlers,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.read = default_flash_read,
	.probe = stm32x_probe,
	.auto_probe = stm32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32x_protect_check,
	.info = get_stm32x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *
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
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/powerpc.h>
#include "spc58x.h"
#include "../../../contrib/loaders/flash/powerpc/spc58x.inc"

#define C_REG_BASE                 0xC3F88000
#define C_ARRAY_BASE               0x00000000
#define SHADOW_ROW_BASE            0x00200000
#define SHADOW_ROW_SIZE            0x00004000
#define FLASH_PAGE_SIZE    C90FL_PAGE_SIZE_08

typedef enum _c55_page_size
{
    C55_PAGE_SIZE_08  = 0x08,
    C55_PAGE_SIZE_16  = 0x10,
    C55_PAGE_SIZE_32  = 0x20
}  C55_PAGE_SIZE_TYPE;


struct spc58xc_flash_bank {
	int probed;
	uint32_t user_bank_addr;
	uint32_t user_bank_size;

	uint32_t low_max_index;
	uint32_t mid_max_index;
	uint32_t high_max_index;
	uint32_t large_max_index;

	SSD_CONFIG ssd;
};

/* flash bank spc58xc <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(spc58xc_flash_bank_command)
{
	struct spc58xc_flash_bank *spc58xc_info;

	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	spc58xc_info = malloc(sizeof(struct spc58xc_flash_bank));
	bank->driver_priv = spc58xc_info;

	spc58xc_info->probed = 0;
	spc58xc_info->user_bank_addr = bank->base;
	spc58xc_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static int spc58xc_protect_check(struct flash_bank *bank)
{
	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	return ERROR_OK;
}

static int spc58xc_setlock(struct flash_bank *bank, uint32_t block_space, uint32_t lock_state)
{
	int err;
	struct target *target = bank->target;
	struct spc58xc_flash_bank *spc58xc_info = bank->driver_priv;
	struct working_area *ssd_config;
	struct working_area *setlock_algorithm;
	struct reg_param reg_params[5];
	struct powerpc_algorithm powerpc_info;
	SSD_CONFIG *ssd = &spc58xc_info->ssd;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _c55_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;

	/* Flash erase code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_setlock_code),
			&setlock_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, setlock_algorithm->address,
			sizeof(spc58x_flash_setlock_code), (uint8_t *)spc58x_flash_setlock_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, setlock_algorithm);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

	init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
	buf_set_u32(reg_params[1].value, 0, 32, block_space);

	init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
	buf_set_u32(reg_params[2].value, 0, 32, lock_state);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[3], "lr", 32, PARAM_IN);
	buf_set_u32(reg_params[3].value, 0, 32, setlock_algorithm->address + (sizeof(spc58x_flash_setlock_code) - 2));

	init_reg_param(&reg_params[4], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[4].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


	err = target_run_algorithm(target,
			0, NULL,
			5, reg_params,
			setlock_algorithm->address, (setlock_algorithm->address + setlock_algorithm->size),
			5000, &powerpc_info);

	//if ((err != ERROR_OK) || (buf_get_u32(reg_params[0].value, 0, 32) != 0)) {
	if (err != ERROR_OK) {
		LOG_INFO("UHHHHHH");
		err = ERROR_TARGET_FAILURE;
	}

	/* Free resources */
	target_free_working_area(target, ssd_config);
	target_free_working_area(target, setlock_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return err;
}

static int spc58xc_getlock(struct flash_bank *bank,
		uint8_t block_space, uint32_t *lock_state)
{
	int err;
	struct target *target = bank->target;
	struct spc58xc_flash_bank *spc58xc_info = bank->driver_priv;
	struct working_area *ssd_config;

	struct working_area *getlock_working_area;

	struct working_area *getlock_algorithm;
	struct reg_param reg_params[5];
	struct powerpc_algorithm powerpc_info;
	SSD_CONFIG *ssd = &spc58xc_info->ssd;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _c55_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;

	if (target_alloc_working_area(target, 4,
			&getlock_working_area) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do get lock working area allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	uint32_t tmp[1]={0};

	err = target_write_buffer(target, getlock_working_area->address,
			4, (uint8_t *)tmp);
	if (err != ERROR_OK)
		return err;


	/* Flash getlock code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_getlock_code),
			&getlock_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, getlock_working_area);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, getlock_algorithm->address,
			sizeof(spc58x_flash_getlock_code), (uint8_t *)spc58x_flash_getlock_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, getlock_algorithm);
		target_free_working_area(target, getlock_working_area);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);
	LOG_DEBUG("------> R3= 0x%08x",buf_get_u32(reg_params[0].value, 0, 32));

	init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
	buf_set_u32(reg_params[1].value, 0, 32, block_space);

	init_reg_param(&reg_params[2], "r5", 32, PARAM_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, getlock_working_area->address);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[3], "lr", 32, PARAM_IN);
	buf_set_u32(reg_params[3].value, 0, 32, getlock_algorithm->address + (sizeof(spc58x_flash_getlock_code) - 2));

	init_reg_param(&reg_params[4], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[4].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


	err = target_run_algorithm(target,
			0, NULL,
			5, reg_params,
			getlock_algorithm->address, (getlock_algorithm->address + getlock_algorithm->size),
			5000, &powerpc_info);

	if (err != ERROR_OK) {
		err = ERROR_TARGET_FAILURE;
		goto flash_getlock_error;
	}
#if 1
	if(buf_get_u32(reg_params[0].value, 0, 32) != 0)
	{
		err = ERROR_TARGET_FAILURE;
		LOG_INFO("Error 2: return value=0x%08x",buf_get_u32(reg_params[0].value, 0, 32));
		goto flash_getlock_error;
	}
#endif

	err = target_read_buffer(target, getlock_working_area->address,
			4, (uint8_t *)tmp);
	if (err != ERROR_OK) {
		goto flash_getlock_error;
	}


	/*  */
	*lock_state = fast_target_buffer_get_u32(&tmp[0], false);

	LOG_DEBUG("GetLock OK: return value (R3)=0x%08x lock_state=%d",buf_get_u32(reg_params[0].value, 0, 32), *lock_state);
flash_getlock_error:
	/* Free resources */
	target_free_working_area(target, ssd_config);
	target_free_working_area(target, getlock_algorithm);
	target_free_working_area(target, getlock_working_area);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return err;
}


static int spc58xc_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{

	unsigned int i;
	int err;
	struct target *target = bank->target;
	struct spc58xc_flash_bank *spc58xc_info = bank->driver_priv;
	struct working_area *ssd_config;
	struct working_area *erase_algorithm;
	struct working_area *erase_working_area;

	struct working_area *checkStatus_algorithm;
	struct working_area *checkStatus_working_area;
	struct working_area *checkStatus_working_area_CtxData;

	struct reg_param reg_params[8];
	struct powerpc_algorithm powerpc_info;

	uint32_t opResult;
	uint32_t opReturn;


	SSD_CONFIG *ssd = &spc58xc_info->ssd;

	LOG_INFO("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t low_mask = 0;
	uint32_t mid_mask = 0;
	uint32_t high_mask = 0;
	NLARGE_BLOCK_SEL nLargeBlockSelect;
	nLargeBlockSelect.firstLargeBlockSelect = 0;
	nLargeBlockSelect.secondLargeBlockSelect = 0;
	uint32_t firstLargeBlockSelect = 0;
	uint32_t secondLargeBlockSelect = 0;


	uint8_t LOW_LOCK_SEL_CODE[8] = {3, 1, 4, 2, 6, 7, 8, 9};
	// uint8_t LARGE_LOCK_SEL_CODE[8] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
	uint8_t LOW_LOCK_SEL_SECURITY[3]= {5, 10, 11};

	if (bank->base == 0xFC0000)
	{
		for (i = first; i <= last; i++) {
			if (i < spc58xc_info->low_max_index) {
				low_mask |= (1 << LOW_LOCK_SEL_CODE[i]);
			} else if (i < spc58xc_info->large_max_index) {
				nLargeBlockSelect.firstLargeBlockSelect |= (1 << (i - spc58xc_info->low_max_index));
			}
		}
	}
	else if (bank->base == 0x60C000)
	{
		for (i = first; i <= last; i++) {
			low_mask |= (1 << LOW_LOCK_SEL_SECURITY[i]);
		}
	}
	else if (bank->base == 0x680000)
	{
		for (i = first; i <= last; i++) {
			if (i < spc58xc_info->mid_max_index) {
				mid_mask |= (1 << i);
			}
		}
	}
	else if (bank->base == 0x800000)
	{
		for (i = first; i <= last; i++) {
			if (i < spc58xc_info->high_max_index) {
				high_mask |= (1 << i);
			}
		}
	}


	/* unlock flash registers */
	uint32_t lock_state;

	if (low_mask != 0) {
		err = spc58xc_getlock(bank, C55_BLOCK_LOW, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc58xc_setlock(bank, C55_BLOCK_LOW, (lock_state & 0xFFFFF000));
		if (err != ERROR_OK)
			return err;
	}

	if (mid_mask != 0) {
		err = spc58xc_getlock(bank, C55_BLOCK_MID,  &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc58xc_setlock(bank, C55_BLOCK_MID, (lock_state & 0xFFFFFFFC));
		if (err != ERROR_OK)
			return err;
	}

	if (high_mask != 0) {
		err = spc58xc_getlock(bank, C55_BLOCK_HIGH, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc58xc_setlock(bank, C55_BLOCK_HIGH, (lock_state & 0xFFFFFFF0));
		if (err != ERROR_OK)
			return err;
	}

	if ((nLargeBlockSelect.firstLargeBlockSelect != 0) || (nLargeBlockSelect.secondLargeBlockSelect != 0)) {
		err = spc58xc_getlock(bank, C55_BLOCK_LARGE_FIRST, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc58xc_setlock(bank, C55_BLOCK_LARGE_FIRST, (lock_state & 0xFFFF0000));
		if (err != ERROR_OK)
			return err;

		firstLargeBlockSelect = nLargeBlockSelect.firstLargeBlockSelect;
		secondLargeBlockSelect = nLargeBlockSelect.secondLargeBlockSelect;

		nLargeBlockSelect.firstLargeBlockSelect = fast_target_buffer_get_u32((uint32_t *)&firstLargeBlockSelect, false);
		nLargeBlockSelect.secondLargeBlockSelect = fast_target_buffer_get_u32((uint32_t *)&secondLargeBlockSelect, false);
	}

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _c55_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;


	/* Flash erase code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_erase_code),
			&erase_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, erase_algorithm->address,
			sizeof(spc58x_flash_erase_code), (uint8_t *)spc58x_flash_erase_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, erase_algorithm);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

	/* eraseOption C55_ERASE_MAIN (0x0) */
	init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
	buf_set_u32(reg_params[1].value, 0, 32, 0);

	/* lowBlockSelect */
	init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
	buf_set_u32(reg_params[2].value, 0, 32, low_mask);

	/* midBlockSelect */
	init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
	buf_set_u32(reg_params[3].value, 0, 32, mid_mask);

	/* highBlockSelect */
	init_reg_param(&reg_params[4], "r7", 32, PARAM_IN);
	buf_set_u32(reg_params[4].value, 0, 32, high_mask);

	/* nLargeBlockSelect */
	if (target_alloc_working_area(target, 8,
			&erase_working_area) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do get lock working area allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, erase_working_area->address,
			8, (uint8_t *)&nLargeBlockSelect);
	if (err != ERROR_OK)
		return err;

	init_reg_param(&reg_params[5], "r8", 32, PARAM_IN);
	buf_set_u32(reg_params[5].value, 0, 32, erase_working_area->address);


	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[6], "lr", 32, PARAM_IN);
	buf_set_u32(reg_params[6].value, 0, 32, erase_algorithm->address + (sizeof(spc58x_flash_erase_code) - 2));

	init_reg_param(&reg_params[7], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[7].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


	err = target_run_algorithm(target,
			0, NULL,
			8, reg_params,
			erase_algorithm->address, (erase_algorithm->address + erase_algorithm->size),
			5000, &powerpc_info);

	if ((err != ERROR_OK) || (buf_get_u32(reg_params[0].value, 0, 32) != 0)) {
		err = ERROR_TARGET_FAILURE;

		target_free_working_area(target, erase_algorithm);
		target_free_working_area(target, erase_working_area);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
		destroy_reg_param(&reg_params[6]);
		destroy_reg_param(&reg_params[7]);
		goto flash_erase_error;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);
	destroy_reg_param(&reg_params[7]);

/* Check Status */
	opResult=0;
	opReturn=C55_INPROGRESS;

	/* Flash checkStatus code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_CheckStatus_code),
				&checkStatus_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash check status during erase step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, checkStatus_algorithm->address,
				sizeof(spc58x_flash_CheckStatus_code), (uint8_t *)spc58x_flash_CheckStatus_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, checkStatus_algorithm);

		target_free_working_area(target, erase_algorithm);
		target_free_working_area(target, erase_working_area);
		return err;
	}

	/* CtxData */
	if (target_alloc_working_area(target, sizeof(struct _c55_context_data),
			&checkStatus_working_area_CtxData) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do get lock working area allocation");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, checkStatus_algorithm);

		target_free_working_area(target, erase_algorithm);
		target_free_working_area(target, erase_working_area);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* opResult */
	if (target_alloc_working_area(target, 8,
			&checkStatus_working_area) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do get lock working area allocation");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, checkStatus_algorithm);

		target_free_working_area(target, erase_algorithm);
		target_free_working_area(target, erase_working_area);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	while (opReturn == C55_INPROGRESS)
	{
		/* ssd_config (in), return value (out) */
		init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* modeOp C55_MODE_OP_ERASE (0x1) */
		init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
		buf_set_u32(reg_params[1].value, 0, 32, C55_MODE_OP_ERASE);

		opResult = 0;
		err = target_write_buffer(target, checkStatus_working_area->address,
				8, (uint8_t *)&opResult);
		if (err != ERROR_OK){
			target_free_working_area(target, ssd_config);
			target_free_working_area(target, checkStatus_algorithm);

			target_free_working_area(target, erase_algorithm);
			target_free_working_area(target, erase_working_area);

			return err;
		}

		/* opResult */
		init_reg_param(&reg_params[2], "r5", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[2].value, 0, 32, checkStatus_working_area->address);

/*
		err = target_write_buffer(target, checkStatus_working_area_CtxData->address,
				sizeof(_c55_context_data), (uint8_t *)&opResult);
		if (err != ERROR_OK)
			return err;
*/
		init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
		buf_set_u32(reg_params[3].value, 0, 32, checkStatus_working_area_CtxData->address);


		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[4], "lr", 32, PARAM_IN);
		buf_set_u32(reg_params[4].value, 0, 32, checkStatus_algorithm->address + (sizeof(spc58x_flash_CheckStatus_code) - 2));

		init_reg_param(&reg_params[5], "r1", 32, PARAM_IN);
		buf_set_u32(reg_params[5].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


		err = target_run_algorithm(target,
				0, NULL,
				6, reg_params,
				checkStatus_algorithm->address, (checkStatus_algorithm->address + checkStatus_algorithm->size),
				5000, &powerpc_info);




		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);
			target_free_working_area(target, checkStatus_working_area_CtxData);

			target_free_working_area(target, erase_algorithm);
			target_free_working_area(target, erase_working_area);

			destroy_reg_param(&reg_params[0]);
			destroy_reg_param(&reg_params[1]);
			destroy_reg_param(&reg_params[2]);
			destroy_reg_param(&reg_params[3]);
			destroy_reg_param(&reg_params[4]);
			destroy_reg_param(&reg_params[5]);

			goto flash_erase_error;
		}

		opReturn = buf_get_u32(reg_params[0].value, 0, 32);
	}

	err = target_read_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);

	err = target_read_buffer(target, checkStatus_working_area->address,
				8, (uint8_t *)&opResult);

	if (opResult == C55_OK)
	{
		/* lock flash registers */
		for (i = first; i <= last; i++) {
			bank->sectors[i].is_erased = 1;
		}
	}
	else
	{
		err = ERROR_TARGET_FAILURE;
	}

	target_free_working_area(target, erase_algorithm);
	target_free_working_area(target, checkStatus_algorithm);
	target_free_working_area(target, checkStatus_working_area);
	target_free_working_area(target, checkStatus_working_area_CtxData);

	target_free_working_area(target, erase_working_area);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);


flash_erase_error:
	/* Free resources */
	target_free_working_area(target, ssd_config);

	return err;
}

static int spc58xc_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

#if 0
/*offset = base address count = size */
static int spc58xc_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	LOG_INFO("%s:%d %s()", __FILE__, __LINE__, __func__);
	LOG_INFO("%s:%d %s() offset = 0x%08x count = 0x%08x", __FILE__, __LINE__, __func__, offset, count);

	unsigned int i, sector = 0;

	for(i=0; i < bank->num_sectors; i++)
		LOG_DEBUG("-----> bank->sectors[%d].offset = 0x%08x (size = %d K)", i, bank->sectors[i].offset, (bank->sectors[i].size/1024));

	struct powerpc_algorithm powerpc_info;
	struct target *target = bank->target;
	struct spc58xc_flash_bank *spc58xc_info = bank->driver_priv;

	static struct working_area *source;
	struct working_area *write_algorithm;
	struct working_area *ssd_config;
	struct working_area *write_working_area_CtxData;
	struct working_area *checkStatus_algorithm;
	struct working_area *checkStatus_working_area;

	struct reg_param reg_params[8];
	SSD_CONFIG *ssd = &spc58x_info->ssd;
	int err = ERROR_OK;


	uint32_t chunk_number;
	uint32_t bytes_remain;
	int32_t tot_sector = 0;
	uint32_t size = 0;
	uint32_t chunk_size = 0x200; /* internal buffer size 512 bytes */

	uint32_t opResult;
	uint32_t opReturn;

	uint32_t loc_func;

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

    if (count > chunk_size)
    {
    	size = chunk_size;
    	bytes_remain = count % chunk_size;
    	chunk_number = count / chunk_size;
    }
    else
    {
    	size = count;
    	bytes_remain = 0;
    	chunk_number = 1;
    }



	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}


	for(i = 0; i < bank->num_sectors; i++)
	{
		if((offset >= bank->sectors[i].offset) && (offset < bank->sectors[i+1].offset)) {
			/* sector found */
			sector = i;
			tot_sector++;
			LOG_INFO("Sector found: %d IN_offset= 0x%08x, bank->sectors[%d].offset= 0x%08x, bank->sectors[%d].size= 0x%08x",
						  sector, offset, i, bank->sectors[i].offset, i, bank->sectors[i].size);

			LOG_INFO("bank->sectors[%d].size = %d",sector, bank->sectors[sector].size);
			LOG_INFO("bank->sectors[%d].offset = 0x%08x", sector, bank->sectors[sector].offset);
			LOG_INFO("bank->sectors[%d].is_erased = %d",sector, bank->sectors[sector].is_erased);
			LOG_INFO("bank->sectors[%d].is_protected = %d", sector, bank->sectors[sector].is_protected);

			/* how many sectors */
			if(count > bank->sectors[sector].size)
			{
				uint32_t next_sect = sector;
				do
				{
					tot_sector++;
					next_sect++;
				}while(count > (bank->sectors[next_sect].offset + bank->sectors[next_sect].size));
			}
			break;
		}
	}



	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _c55_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;

	/* unlock flash registers */
    // LOG_INFO("----> UNLOCK Flash");


	/* flash write code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	//pgmCtxData.pReqCompletionFn =  (PFLASHPROGRAM) write_algorithm->address;
	//buf_set_u32((uint8_t *)&loc_func, 0, 32,  buf_get_u32((uint8_t *) &(write_algorithm->address), 0, 32));

	loc_func = fast_target_buffer_get_u32((uint8_t *)&write_algorithm->address, false);

	err = target_write_buffer(target, write_algorithm->address,
			sizeof(spc58x_flash_write_code), (uint8_t *)spc58x_flash_write_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		return err;
	}

	/* memory buffer */
	if (target_alloc_working_area_try(target, chunk_size, &source) != ERROR_OK) {
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	/*if (target_alloc_working_area_try(target, sizeof(CONTEXT_DATA), &write_working_area_CtxData) != ERROR_OK) { */
	if (target_alloc_working_area_try(target, 36, &write_working_area_CtxData) != ERROR_OK) {
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

/*	err = target_write_buffer(target, write_working_area_CtxData->address,
			sizeof(CONTEXT_DATA), (uint8_t *)&pgmCtxData);


	err = powerpc_write_memory_noturn(target, write_working_area_CtxData->address + 32,
			4, (uint8_t *)&write_algorithm->address);
*/
	err = target_write_buffer(target, write_working_area_CtxData->address + 32,
				32, (uint8_t *)&loc_func);

	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		return err;
	}

	/* Flash checkStatus code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_CheckStatus_code),
				&checkStatus_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash check status during erase step");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
    	target_free_working_area(target, source);
	    target_free_working_area(target, write_working_area_CtxData);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, checkStatus_algorithm->address,
				sizeof(spc58x_flash_CheckStatus_code), (uint8_t *)spc58x_flash_CheckStatus_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		target_free_working_area(target, checkStatus_algorithm);
		return err;
	}

	/* opResult */
	if (target_alloc_working_area(target, 4,
			&checkStatus_working_area) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do get lock working area allocation");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		target_free_working_area(target, checkStatus_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	opResult = 0;
	err = target_write_buffer(target, checkStatus_working_area->address,
			4, (uint8_t *)&opResult);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		target_free_working_area(target, checkStatus_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(i = 0; i < chunk_number; i++)
	{

		err = target_write_buffer(target, source->address,
				size, (uint8_t *)(buffer  + (i * chunk_size)));
		if (err != ERROR_OK) {
			target_free_working_area(target, ssd_config);
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);
			return err;
		}

		init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* factoryPgmFlag  - FALSE to do normal program. */
		init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
		buf_set_u32(reg_params[1].value, 0, 32, 0);

		/* dest */
		init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
		buf_set_u32(reg_params[2].value, 0, 32, bank->base + offset + (i * chunk_size));
		/* buf_set_u32(reg_params[2].value, 0, 32, 0xFC0000); */

		/* size */
		init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);	/* size = 0x200 */
		buf_set_u32(reg_params[3].value, 0, 32, size);

		init_reg_param(&reg_params[4], "r7", 32, PARAM_IN);	/* source */
		buf_set_u32(reg_params[4].value, 0, 32, source->address);

		init_reg_param(&reg_params[5], "r8", 32, PARAM_IN);
		buf_set_u32(reg_params[5].value, 0, 32, write_working_area_CtxData->address);

		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[6], "lr", 32, PARAM_IN);	/* lr */
		buf_set_u32(reg_params[6].value, 0, 32, write_algorithm->address +(sizeof(spc58x_flash_write_code) - 2));

		init_reg_param(&reg_params[7], "r1", 32, PARAM_IN);
		buf_set_u32(reg_params[7].value, 0, 32, 0x400AFFFF);


		err = target_run_algorithm(target,
				0, NULL,
				8, reg_params,
				write_algorithm->address, (write_algorithm->address + write_algorithm->size),
				5000, &powerpc_info);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
		destroy_reg_param(&reg_params[6]);
		destroy_reg_param(&reg_params[7]);


		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);

			goto flash_write_error;
		}

/* Check Status */
		opResult=0;
		opReturn=C55_INPROGRESS;

		while (opReturn == C55_INPROGRESS)
		{
			/* ssd_config (in), return value (out) */
			init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
			buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

			/* modeOp C55_MODE_OP_ERASE (0x1) */
			init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
			buf_set_u32(reg_params[1].value, 0, 32, C55_MODE_OP_PROGRAM);

			/* opResult */
			init_reg_param(&reg_params[2], "r5", 32, PARAM_IN_OUT);
			buf_set_u32(reg_params[2].value, 0, 32, checkStatus_working_area->address);

			init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
			buf_set_u32(reg_params[3].value, 0, 32, write_working_area_CtxData->address);

			/*
			 * Link register (in).
			 * Set link register to the breakpoint instruction at the end of the buffer.
			 * We use a software breakpoint to notify when done with algorithm execution.
			 */
			init_reg_param(&reg_params[4], "lr", 32, PARAM_IN);
			buf_set_u32(reg_params[4].value, 0, 32, checkStatus_algorithm->address + (sizeof(spc58x_flash_CheckStatus_code) - 2));

			init_reg_param(&reg_params[5], "r1", 32, PARAM_IN);
			buf_set_u32(reg_params[5].value, 0, 32, 0x400AFFFF);


			err = target_run_algorithm(target,
					0, NULL,
					6, reg_params,
					checkStatus_algorithm->address, (checkStatus_algorithm->address + checkStatus_algorithm->size),
					2000000000, &powerpc_info);


			if (err != ERROR_OK)  {
				err = ERROR_TARGET_FAILURE;
				target_free_working_area(target, write_algorithm);
				target_free_working_area(target, source);
				target_free_working_area(target, write_working_area_CtxData);
				target_free_working_area(target, checkStatus_algorithm);
				target_free_working_area(target, checkStatus_working_area);

				destroy_reg_param(&reg_params[0]);
				destroy_reg_param(&reg_params[1]);
				destroy_reg_param(&reg_params[2]);
				destroy_reg_param(&reg_params[3]);
				destroy_reg_param(&reg_params[4]);
				destroy_reg_param(&reg_params[5]);

				goto flash_write_error;
			}

			opReturn = buf_get_u32(reg_params[0].value, 0, 32);
		}

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
	}

	if(bytes_remain)
	{

		err = target_write_buffer(target, source->address,
				bytes_remain, (uint8_t *)(buffer  + (i * chunk_size)));
		if (err != ERROR_OK) {
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);
			return err;
		}



		init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* factoryPgmFlag  - FALSE to do normal program. */
		init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
		buf_set_u32(reg_params[1].value, 0, 32, 0);

		/* dest */
		init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
		buf_set_u32(reg_params[2].value, 0, 32, bank->base + offset + (i * chunk_size));

		/* size */
		init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
		buf_set_u32(reg_params[3].value, 0, 32, bytes_remain);

		init_reg_param(&reg_params[4], "r7", 32, PARAM_IN);	/* source */
		buf_set_u32(reg_params[4].value, 0, 32, source->address);

		init_reg_param(&reg_params[5], "r8", 32, PARAM_IN);
		buf_set_u32(reg_params[5].value, 0, 32, write_working_area_CtxData->address);

		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[6], "lr", 32, PARAM_IN);	/* lr */
		buf_set_u32(reg_params[6].value, 0, 32, write_algorithm->address +(sizeof(spc58x_flash_write_code) - 2));

		init_reg_param(&reg_params[7], "r1", 32, PARAM_IN);
		buf_set_u32(reg_params[7].value, 0, 32, 0x400AFFFF);

		err = target_run_algorithm(target,
				0, NULL,
				8, reg_params,
				write_algorithm->address, (write_algorithm->address + write_algorithm->size),
				5000, &powerpc_info);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
		destroy_reg_param(&reg_params[6]);
		destroy_reg_param(&reg_params[7]);

		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);
			goto flash_write_error;
		}

/* Check Status */
		opResult=0;
		opReturn=C55_INPROGRESS;

		while (opReturn == C55_INPROGRESS)
		{
			/* ssd_config (in), return value (out) */
			init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
			buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

			/* modeOp C55_MODE_OP_ERASE (0x1) */
			init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
			buf_set_u32(reg_params[1].value, 0, 32, C55_MODE_OP_PROGRAM);

			opResult = 0;
			err = target_write_buffer(target, checkStatus_working_area->address,
					4, (uint8_t *)&opResult);
			if (err != ERROR_OK)
				return err;

			/* opResult */
			init_reg_param(&reg_params[2], "r5", 32, PARAM_IN_OUT);
			buf_set_u32(reg_params[2].value, 0, 32,  checkStatus_working_area->address);

			init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
			buf_set_u32(reg_params[3].value, 0, 32, write_working_area_CtxData->address);

			/*
			 * Link register (in).
			 * Set link register to the breakpoint instruction at the end of the buffer.
			 * We use a software breakpoint to notify when done with algorithm execution.
			 */
			init_reg_param(&reg_params[4], "lr", 32, PARAM_IN);
			buf_set_u32(reg_params[4].value, 0, 32, checkStatus_algorithm->address + (sizeof(spc58x_flash_CheckStatus_code) - 2));

			init_reg_param(&reg_params[5], "r1", 32, PARAM_IN);
			buf_set_u32(reg_params[5].value, 0, 32, 0x400AFFFF);


			err = target_run_algorithm(target,
					0, NULL,
					6, reg_params,
					checkStatus_algorithm->address, (checkStatus_algorithm->address + checkStatus_algorithm->size),
					2000000000, &powerpc_info);


			if (err != ERROR_OK)  {
				err = ERROR_TARGET_FAILURE;
				target_free_working_area(target, checkStatus_algorithm);
				target_free_working_area(target, checkStatus_working_area);

				destroy_reg_param(&reg_params[0]);
				destroy_reg_param(&reg_params[1]);
				destroy_reg_param(&reg_params[2]);
				destroy_reg_param(&reg_params[3]);
				destroy_reg_param(&reg_params[4]);
				destroy_reg_param(&reg_params[5]);

				goto flash_write_error;
			}

			opReturn = buf_get_u32(reg_params[0].value, 0, 32);
		}
	}

	err = target_read_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);

	err = target_read_buffer(target, checkStatus_working_area->address,
				8, (uint8_t *)&opResult);

	if ((err != ERROR_OK) || (opResult != C55_OK))
	{
		err = ERROR_TARGET_FAILURE;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, write_working_area_CtxData);
	target_free_working_area(target, checkStatus_algorithm);
	target_free_working_area(target, checkStatus_working_area);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);


flash_write_error:
	/* Free resources */
	target_free_working_area(target, ssd_config);

	return err;
}
#endif

/*offset = base address count = size */
static int spc58xc_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	LOG_INFO("%s:%d %s()", __FILE__, __LINE__, __func__);
	LOG_INFO("%s:%d %s() offset = 0x%08x count = 0x%08x", __FILE__, __LINE__, __func__, offset, count);

	unsigned int i, sector = 0;

	for(i=0; i < bank->num_sectors; i++)
		LOG_DEBUG("-----> bank->sectors[%d].offset = 0x%08x (size = %d K)", i, bank->sectors[i].offset, (bank->sectors[i].size/1024));

	struct powerpc_algorithm powerpc_info;
	struct target *target = bank->target;
	struct spc58xc_flash_bank *spc58xc_info = bank->driver_priv;

	static struct working_area *source;
	struct working_area *write_algorithm;
	struct working_area *ssd_config;
	struct working_area *write_working_area_CtxData;
	struct working_area *checkStatus_algorithm;
	struct working_area *checkStatus_working_area;

	struct reg_param reg_params[8];
	SSD_CONFIG *ssd = &spc58xc_info->ssd;
	int err = ERROR_OK;


	uint32_t chunk_number;
	uint32_t bytes_remain;
	int32_t tot_sector = 0;
	uint32_t size = 0;
	uint32_t chunk_size = 0x400; /* internal buffer size 1024 bytes */

	uint32_t opResult;


	uint32_t loc_func;

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

    if (count > chunk_size)
    {
    	size = chunk_size;
    	bytes_remain = count % chunk_size;
    	chunk_number = count / chunk_size;
    }
    else
    {
    	size = count;
    	bytes_remain = 0;
    	chunk_number = 1;
    }



	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}


	for(i = 0; i < bank->num_sectors; i++)
	{
		if((offset >= bank->sectors[i].offset) && (offset < bank->sectors[i+1].offset)) {
			/* sector found */
			sector = i;
			tot_sector++;
			LOG_INFO("Sector found: %d IN_offset= 0x%08x, bank->sectors[%d].offset= 0x%08x, bank->sectors[%d].size= 0x%08x",
						  sector, offset, i, bank->sectors[i].offset, i, bank->sectors[i].size);

			LOG_INFO("bank->sectors[%d].size = %d",sector, bank->sectors[sector].size);
			LOG_INFO("bank->sectors[%d].offset = 0x%08x", sector, bank->sectors[sector].offset);
			LOG_INFO("bank->sectors[%d].is_erased = %d",sector, bank->sectors[sector].is_erased);
			LOG_INFO("bank->sectors[%d].is_protected = %d", sector, bank->sectors[sector].is_protected);

			/* how many sectors */
			if(count > bank->sectors[sector].size)
			{
				uint32_t next_sect = sector;
				do
				{
					tot_sector++;
					next_sect++;
				}while(count > (bank->sectors[next_sect].offset + bank->sectors[next_sect].size));
			}
			break;
		}
	}



	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _c55_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;

	/* unlock flash registers */
    // LOG_INFO("----> UNLOCK Flash");


	/* flash write code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	//pgmCtxData.pReqCompletionFn =  (PFLASHPROGRAM) write_algorithm->address;
	//buf_set_u32((uint8_t *)&loc_func, 0, 32,  buf_get_u32((uint8_t *) &(write_algorithm->address), 0, 32));

	loc_func = fast_target_buffer_get_u32((uint8_t *)&write_algorithm->address, false);

	err = target_write_buffer(target, write_algorithm->address,
			sizeof(spc58x_flash_write_code), (uint8_t *)spc58x_flash_write_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		return err;
	}

	/* memory buffer */
	if (target_alloc_working_area_try(target, chunk_size, &source) != ERROR_OK) {
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	/*if (target_alloc_working_area_try(target, sizeof(CONTEXT_DATA), &write_working_area_CtxData) != ERROR_OK) { */
	if (target_alloc_working_area_try(target, 36, &write_working_area_CtxData) != ERROR_OK) {
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

/*	err = target_write_buffer(target, write_working_area_CtxData->address,
			sizeof(CONTEXT_DATA), (uint8_t *)&pgmCtxData);


	err = powerpc_write_memory_noturn(target, write_working_area_CtxData->address + 32,
			4, (uint8_t *)&write_algorithm->address);
*/
	err = target_write_buffer(target, write_working_area_CtxData->address + 32,
				32, (uint8_t *)&loc_func);

	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		return err;
	}

	/* Flash checkStatus code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_CheckStatus_code),
				&checkStatus_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash check status during erase step");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
    	target_free_working_area(target, source);
	    target_free_working_area(target, write_working_area_CtxData);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, checkStatus_algorithm->address,
				sizeof(spc58x_flash_CheckStatus_code), (uint8_t *)spc58x_flash_CheckStatus_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		target_free_working_area(target, checkStatus_algorithm);
		return err;
	}

	/* opResult */
	if (target_alloc_working_area(target, 4,
			&checkStatus_working_area) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do get lock working area allocation");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		target_free_working_area(target, checkStatus_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	opResult = 0;
	err = target_write_buffer(target, checkStatus_working_area->address,
			4, (uint8_t *)&opResult);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_working_area_CtxData);
		target_free_working_area(target, checkStatus_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(i = 0; i < chunk_number; i++)
	{

		err = target_write_buffer(target, source->address,
				size, (uint8_t *)(buffer  + (i * chunk_size)));
		if (err != ERROR_OK) {
			target_free_working_area(target, ssd_config);
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);
			return err;
		}

		init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* factoryPgmFlag  - FALSE to do normal program. */
		init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
		buf_set_u32(reg_params[1].value, 0, 32, 0);

		/* dest */
		init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
		buf_set_u32(reg_params[2].value, 0, 32, bank->base + offset + (i * chunk_size));
		/* buf_set_u32(reg_params[2].value, 0, 32, 0xFC0000); */

		/* size */
		init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);	/* size = 0x200 */
		buf_set_u32(reg_params[3].value, 0, 32, size);

		init_reg_param(&reg_params[4], "r7", 32, PARAM_IN);	/* source */
		buf_set_u32(reg_params[4].value, 0, 32, source->address);

		init_reg_param(&reg_params[5], "r8", 32, PARAM_IN);
		buf_set_u32(reg_params[5].value, 0, 32, write_working_area_CtxData->address);


		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[6], "lr", 32, PARAM_IN);	/* lr */
		buf_set_u32(reg_params[6].value, 0, 32, write_algorithm->address +(sizeof(spc58x_flash_write_code) - 2));

		init_reg_param(&reg_params[7], "r1", 32, PARAM_IN);
		buf_set_u32(reg_params[7].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


		err = target_run_algorithm(target,
				0, NULL,
				8, reg_params,
				write_algorithm->address, (write_algorithm->address + write_algorithm->size),
				2000000000, &powerpc_info);

		LOG_DEBUG("Device buffer Size: %d, Number of iteraction: %d, Current iteraction: %d, err: %d", chunk_size, chunk_number, i, err);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
		destroy_reg_param(&reg_params[6]);
		destroy_reg_param(&reg_params[7]);


		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);

			goto flash_write_error;
		}

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
	}

	if(bytes_remain)
	{

		err = target_write_buffer(target, source->address,
				bytes_remain, (uint8_t *)(buffer  + (i * chunk_size)));
		if (err != ERROR_OK) {
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);
			return err;
		}



		init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* factoryPgmFlag  - FALSE to do normal program. */
		init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
		buf_set_u32(reg_params[1].value, 0, 32, 0);

		/* dest */
		init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
		buf_set_u32(reg_params[2].value, 0, 32, bank->base + offset + (i * chunk_size));

		/* size */
		init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
		buf_set_u32(reg_params[3].value, 0, 32, bytes_remain);

		init_reg_param(&reg_params[4], "r7", 32, PARAM_IN);	/* source */
		buf_set_u32(reg_params[4].value, 0, 32, source->address);

		init_reg_param(&reg_params[5], "r8", 32, PARAM_IN);
		buf_set_u32(reg_params[5].value, 0, 32, write_working_area_CtxData->address);

		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[6], "lr", 32, PARAM_IN);	/* lr */
		buf_set_u32(reg_params[6].value, 0, 32, write_algorithm->address +(sizeof(spc58x_flash_write_code) - 2));

		init_reg_param(&reg_params[7], "r1", 32, PARAM_IN);
		buf_set_u32(reg_params[7].value, 0, 32, target->working_area_phys + target->working_area_size - 1);

		err = target_run_algorithm(target,
				0, NULL,
				8, reg_params,
				write_algorithm->address, (write_algorithm->address + write_algorithm->size),
				5000, &powerpc_info);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
		destroy_reg_param(&reg_params[6]);
		destroy_reg_param(&reg_params[7]);

		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, checkStatus_algorithm);
			target_free_working_area(target, checkStatus_working_area);
			goto flash_write_error;
		}
	}

	err = target_read_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);

	if (err != ERROR_OK)
	{
		err = ERROR_TARGET_FAILURE;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, write_working_area_CtxData);
	target_free_working_area(target, checkStatus_algorithm);
	target_free_working_area(target, checkStatus_working_area);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);


flash_write_error:
	/* Free resources */
	target_free_working_area(target, ssd_config);

	return err;
}


static void setup_sector(struct flash_bank *bank, unsigned int start, unsigned int num, unsigned int size)
{
	unsigned int i;
	for (i = start; i < (start + num) ; i++) {
		assert(i < bank->num_sectors);
		bank->sectors[i].offset = bank->size;
		bank->sectors[i].size = size;
		bank->size += bank->sectors[i].size;
	}
}

static int spc58xc_probe(struct flash_bank *bank)
{
	struct spc58xc_flash_bank *spc58xc_info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *ssd_config;
	struct working_area *init_algorithm;
	struct reg_param reg_params[3];
	struct powerpc_algorithm powerpc_info;
	SSD_CONFIG *ssd = &spc58xc_info->ssd;


	int i;
	int err;
	uint16_t flash_size_in_kb = 0;

	int num_pages;

	num_pages = 0;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	spc58xc_info->probed = 0;

	/* The user sets the size manually */
	if (spc58xc_info->user_bank_size) {
		LOG_DEBUG("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = spc58xc_info->user_bank_size / 1024;
	}

	LOG_INFO("flash: %d kbytes @ 0x%08x", flash_size_in_kb, spc58xc_info->user_bank_addr);


	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _c55_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	/* Default SSD values (keep them in target endianess) */
	uint32_t val;
	val = C55_REG_BASE;
    ssd->c55RegBase = fast_target_buffer_get_u32(&val, false);

    val = MAIN_ARRAY_BASE;
	ssd->mainArrayBase = fast_target_buffer_get_u32(&val, false);

    val = UTEST_ARRAY_BASE;
	ssd->uTestArrayBase = fast_target_buffer_get_u32(&val, false);

	val = C55_PROGRAMMABLE_SIZE;
	ssd->programmableSize = fast_target_buffer_get_u32(&val, false);


	ssd->lowBlockInfo.n16KBlockNum = 0;
	ssd->lowBlockInfo.n32KBlockNum = 0;
	ssd->lowBlockInfo.n64KBlockNum = 0;
	ssd->lowBlockInfo.n128KBlockNum = 0;

	ssd->midBlockInfo.n16KBlockNum = 0;
	ssd->midBlockInfo.n32KBlockNum = 0;
	ssd->midBlockInfo.n64KBlockNum = 0;
	ssd->midBlockInfo.n128KBlockNum = 0;

	ssd->highBlockInfo.n16KBlockNum = 0;
	ssd->highBlockInfo.n32KBlockNum = 0;
	ssd->highBlockInfo.n64KBlockNum = 0;
	ssd->highBlockInfo.n128KBlockNum = 0;

	ssd->nLargeBlockNum = 0;

	ssd->mainInterfaceFlag = 1;

	ssd->BDMEnable = 0;

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
	{
		target_free_working_area(target, ssd_config);
		return err;
	}


	/* Flash initialization code */
	if (target_alloc_working_area(target, sizeof(spc58x_flash_init_code),
			&init_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash init step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, init_algorithm->address,
			sizeof(spc58x_flash_init_code), (uint8_t *)spc58x_flash_init_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, init_algorithm);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[1], "lr", 32, PARAM_IN);
	//buf_set_u32(reg_params[1].value, 0, 32, init_algorithm->address + (init_algorithm->size - 2));
	buf_set_u32(reg_params[1].value, 0, 32, init_algorithm->address + (sizeof(spc58x_flash_init_code) - 2));
	//buf_set_u32(reg_params[1].value, 0, 32, 0x400a8164);

	/*
	* Stack Pointer (in).
	*/
	init_reg_param(&reg_params[2], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[2].value, 0, 32, target->working_area_phys + target->working_area_size - 1);

	err = target_run_algorithm(target,
			0, NULL,
			3, reg_params,
			init_algorithm->address, (init_algorithm->address + init_algorithm->size),
			5000, &powerpc_info);

	if ((err != ERROR_OK) || (buf_get_u32(reg_params[0].value, 0, 32) != 0)) {
		err = ERROR_TARGET_FAILURE;
		goto flash_init_error;
	}

	err = target_read_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK) {
		goto flash_init_error;
	}

	LOG_DEBUG("SDD->c55RegBase  = 0x%08x", fast_target_buffer_get_u32(&ssd->c55RegBase, false));
	LOG_DEBUG("SDD->mainArrayBase = 0x%08x", fast_target_buffer_get_u32(&ssd->mainArrayBase, false));
	LOG_DEBUG("SDD->uTestArrayBase = 0x%08x", fast_target_buffer_get_u32(&ssd->uTestArrayBase, false));

	LOG_DEBUG("SDD->lowBlockInfo->n16KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockInfo.n16KBlockNum, false));
	LOG_DEBUG("SDD->lowBlockInfo->n32KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockInfo.n32KBlockNum, false));
	LOG_DEBUG("SDD->lowBlockInfo->n64KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockInfo.n64KBlockNum, false));
	LOG_DEBUG("SDD->lowBlockInfo->n128KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockInfo.n128KBlockNum, false));

	LOG_DEBUG("SDD->midBlockInfo->n16KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockInfo.n16KBlockNum, false));
	LOG_DEBUG("SDD->midBlockInfo->n32KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockInfo.n32KBlockNum, false));
	LOG_DEBUG("SDD->midBlockInfo->n64KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockInfo.n64KBlockNum, false));
	LOG_DEBUG("SDD->midBlockInfo->n128KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockInfo.n128KBlockNum, false));

	LOG_DEBUG("SDD->highBlockInfo->n16KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockInfo.n16KBlockNum, false));
	LOG_DEBUG("SDD->highBlockInfo->n32KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockInfo.n32KBlockNum, false));
	LOG_DEBUG("SDD->highBlockInfo->n64KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockInfo.n64KBlockNum, false));
	LOG_DEBUG("SDD->highBlockInfo->n128KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockInfo.n128KBlockNum, false));


	LOG_DEBUG("SDD->nLargeBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->nLargeBlockNum, false));

	LOG_DEBUG("SDD->mainInterfaceFlag  = 0x%u", ssd->mainInterfaceFlag);
	LOG_DEBUG("SDD->programmableSize   = 0x%08x", fast_target_buffer_get_u32(&ssd->programmableSize, false));
	LOG_DEBUG("SDD->BDMEnable     = 0x%u", ssd->BDMEnable);


	/* calculate numbers of pages */
	/* int num_pages = fast_target_buffer_get_u32(&ssd->lowBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n128KBlockNum, false);
	num_pages = num_pages + fast_target_buffer_get_u32(&ssd->midBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n128KBlockNum, false);
	num_pages = num_pages + fast_target_buffer_get_u32(&ssd->highBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n128KBlockNum, false);
	num_pages = num_pages + fast_target_buffer_get_u32(&ssd->nLargeBlockNum, false);
	*/
	if(bank->base == 0xFC0000)
	{
		num_pages = 24;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = spc58xc_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);
		setup_sector(bank, 4, 2, 32 * 1024);
		setup_sector(bank, 6, 2, 64 * 1024);

		// Large Flash Blocks
		setup_sector(bank, 8, 2, 128 * 1024);
		setup_sector(bank, 10, 14, 256 * 1024);

		spc58xc_info->low_max_index = 8;
		spc58xc_info->large_max_index = 24;
		spc58xc_info->high_max_index = 0;
		spc58xc_info->mid_max_index = 0;
	}
	else if(bank->base == 0x60C000)
	{
		num_pages = 3;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = spc58xc_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 1, 16 * 1024);
		setup_sector(bank, 1, 2, 64 * 1024);


		spc58xc_info->low_max_index = 3;
		spc58xc_info->large_max_index = 0;
		spc58xc_info->high_max_index = 0;
		spc58xc_info->mid_max_index = 0;
	}
	else if(bank->base == 0x680000)
		{
			num_pages = 2;

			/* check that calculation result makes sense */
			assert(num_pages > 0);

			if (bank->sectors) {
				free(bank->sectors);
				bank->sectors = NULL;
			}

			bank->base = spc58xc_info->user_bank_addr;
			bank->num_sectors = num_pages;
			bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
			bank->size = 0;

			// Mid Flash Blocks
			setup_sector(bank, 0, 2, 16 * 1024);


			spc58xc_info->low_max_index = 0;
			spc58xc_info->large_max_index = 0;
			spc58xc_info->high_max_index = 0;
			spc58xc_info->mid_max_index = 2;
		}
	else if(bank->base == 0x800000)
	{
		num_pages = 4;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = spc58xc_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 32 * 1024);

		spc58xc_info->low_max_index = 0;
		spc58xc_info->large_max_index = 0;
		spc58xc_info->high_max_index = 4;
		spc58xc_info->mid_max_index = 0;
	}



	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}


	/* Save flash geometry (in sectors) */
/*
	spc58xc_info->low_max_index = fast_target_buffer_get_u32(&ssd->lowBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n128KBlockNum, false);
	spc58xc_info->large_max_index = spc58xc_info->low_max_index +  fast_target_buffer_get_u32(&ssd->nLargeBlockNum, false);
	spc58xc_info->high_max_index = spc58xc_info->large_max_index + fast_target_buffer_get_u32(&ssd->highBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n128KBlockNum, false);
	spc58xc_info->mid_max_index = spc58xc_info->high_max_index + fast_target_buffer_get_u32(&ssd->midBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n128KBlockNum, false);
*/



	/* Done */
	spc58xc_info->probed = 1;


flash_init_error:
	/* Free resources */
	target_free_working_area(target, ssd_config);
	target_free_working_area(target, init_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return err;
}

static int spc58xc_auto_probe(struct flash_bank *bank)
{
	struct spc58xc_flash_bank *spc58xc_info = bank->driver_priv;

	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	if (spc58xc_info->probed)
		return ERROR_OK;
	return spc58xc_probe(bank);
}


static int get_spc58xc_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	/* TODO: retrieve the right info */
	//snprintf(buf, buf_size, "SPC560B - Rev: xx");

	return ERROR_OK;
}


struct flash_driver spc58xc_flash = {
	.name = "spc58xc",
	.flash_bank_command = spc58xc_flash_bank_command,
	.erase = spc58xc_erase,
	.protect = spc58xc_protect,
	.write = spc58xc_write,
	.read = default_flash_read,
	.probe = spc58xc_probe,
	.auto_probe = spc58xc_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = spc58xc_protect_check,
	.info = get_spc58xc_info,
};

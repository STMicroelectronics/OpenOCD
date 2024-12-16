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
#include <target/target_type.h>
#include "spc56x.h"
#include "../../../contrib/loaders/flash/powerpc/spc56x.inc"

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


struct spc56x_flash_bank {
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
FLASH_BANK_COMMAND_HANDLER(spc56x_flash_bank_command)
{
	struct spc56x_flash_bank *spc56x_info;

	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	spc56x_info = malloc(sizeof(struct spc56x_flash_bank));
	bank->driver_priv = spc56x_info;

	spc56x_info->probed = 0;
	spc56x_info->user_bank_addr = bank->base;
	spc56x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static int spc56x_protect_check(struct flash_bank *bank)
{
	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	return ERROR_OK;
}

static int spc56x_setlock(struct flash_bank *bank, uint32_t block_space, uint32_t lock_state)
{
	int err;
	uint32_t pass;
	struct target *target = bank->target;
	struct spc56x_flash_bank *spc56x_info = bank->driver_priv;
	struct working_area *ssd_config;
	struct working_area *setlock_algorithm;
	struct reg_param reg_params[6];
	struct powerpc_algorithm powerpc_info;
	SSD_CONFIG *ssd = &spc56x_info->ssd;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;

	/* Flash erase code */
	if (target_alloc_working_area(target, sizeof(spc56x_flash_setlock_code),
			&setlock_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, setlock_algorithm->address,
			sizeof(spc56x_flash_setlock_code), (uint8_t *)spc56x_flash_setlock_code);
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

	if ((block_space == LOCK_LOW_PRIMARY) || (block_space == LOCK_MID_PRIMARY))
	{
		pass = FLASH_LMLR_PASSWORD;
	}
	else if ((block_space == LOCK_LOW_SECONDARY) || (block_space == LOCK_MID_SECONDARY))
	{
		pass = FLASH_SLMLR_PASSWORD;
	}
	else if (block_space == LOCK_HIGH)
	{
		pass = FLASH_HLR_PASSWORD;
	}
	else
	{
		pass = 0x0;
	}


	init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
	buf_set_u32(reg_params[3].value, 0, 32, pass);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[4], "lr", 32, PARAM_IN);
	buf_set_u32(reg_params[4].value, 0, 32, setlock_algorithm->address + (sizeof(spc56x_flash_setlock_code) - 2));

	init_reg_param(&reg_params[5], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[5].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


	err = target_run_algorithm(target,
			0, NULL,
			6, reg_params,
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
	destroy_reg_param(&reg_params[5]);

	return err;
}

static int spc56x_getlock(struct flash_bank *bank,
		uint8_t block_space, uint32_t *lock_state)
{
	int err;
	struct target *target = bank->target;
	struct spc56x_flash_bank *spc56x_info = bank->driver_priv;
	struct working_area *ssd_config;

	struct working_area *getlock_working_area;
	struct working_area *getlock_working_area_blkLockEnabled;

	struct working_area *getlock_algorithm;
	struct reg_param reg_params[6];
	struct powerpc_algorithm powerpc_info;
	SSD_CONFIG *ssd = &spc56x_info->ssd;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _ssd_config),
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

	if (target_alloc_working_area(target, 4,
			&getlock_working_area_blkLockEnabled) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do get lock enabled working area allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	uint32_t tmp1[1]={0};

	err = target_write_buffer(target, getlock_working_area_blkLockEnabled->address,
			4, (uint8_t *)tmp1);
	if (err != ERROR_OK)
		return err;



	/* Flash getlock code */
	if (target_alloc_working_area(target, sizeof(spc56x_flash_getlock_code),
			&getlock_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, getlock_working_area);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, getlock_algorithm->address,
			sizeof(spc56x_flash_getlock_code), (uint8_t *)spc56x_flash_getlock_code);
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
	buf_set_u32(reg_params[2].value, 0, 32, getlock_working_area_blkLockEnabled->address);

	init_reg_param(&reg_params[3], "r6", 32, PARAM_OUT);
	buf_set_u32(reg_params[3].value, 0, 32, getlock_working_area->address);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[4], "lr", 32, PARAM_IN);
	buf_set_u32(reg_params[4].value, 0, 32, getlock_algorithm->address + (sizeof(spc56x_flash_getlock_code) - 2));

	init_reg_param(&reg_params[5], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[5].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


	err = target_run_algorithm(target,
			0, NULL,
			6, reg_params,
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
	target_free_working_area(target, getlock_working_area_blkLockEnabled);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

	return err;
}


static int spc56x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{

	unsigned int i;
	int err;
	struct target *target = bank->target;
	struct spc56x_flash_bank *spc56x_info = bank->driver_priv;
	struct working_area *ssd_config;
	struct working_area *erase_algorithm;

	struct reg_param reg_params[8];
	struct powerpc_algorithm powerpc_info;

	SSD_CONFIG *ssd = &spc56x_info->ssd;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t low_mask = 0;
	uint32_t mid_mask = 0;
	uint32_t high_mask = 0;

	if (bank->base == 0x0)
	{
		for (i = first; i <= last; i++) {
			if (i < spc56x_info->low_max_index) {
				low_mask |= (1 << i);
			} else if (i < spc56x_info->mid_max_index) {
				mid_mask |= (1 << (i - spc56x_info->low_max_index));
			} else if (i < spc56x_info->high_max_index) {
				high_mask |= (1 << (i - spc56x_info->mid_max_index));
			}
		}
	}
	else if (bank->base == 0x180000)
	{
/*
		for (i = first; i <= last; i++) {
			if (i < spc56x_info->high_max_index) {
				high_mask |= (1 << (i + 8));
			}
		}
*/
		for (i = first; i <= last; i++) {
			if (i < spc56x_info->low_max_index) {
				low_mask |= (1 << i);
			} else if (i < spc56x_info->mid_max_index) {
				mid_mask |= (1 << (i - spc56x_info->low_max_index));
			} else if (i < spc56x_info->high_max_index) {
				high_mask |= (1 << (i - spc56x_info->mid_max_index));
			}
		}
	}
	else if (bank->base == 0x800000)
	{
		for (i = first; i <= last; i++) {
			if (i < spc56x_info->low_max_index) {
				low_mask |= (1 << i);
			}
		}
	}


	/* unlock flash registers */
	uint32_t lock_state;

	if (low_mask != 0) {
		err = spc56x_getlock(bank, LOCK_LOW_PRIMARY, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc56x_setlock(bank, LOCK_LOW_PRIMARY, (lock_state & 0xFFFF0000));
		if (err != ERROR_OK)
			return err;

		err = spc56x_getlock(bank, LOCK_LOW_SECONDARY, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc56x_setlock(bank, LOCK_LOW_SECONDARY, (lock_state & 0xFFFF0000));
		if (err != ERROR_OK)
			return err;
	}

	if (mid_mask != 0) {
		err = spc56x_getlock(bank, LOCK_MID_PRIMARY,  &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc56x_setlock(bank, LOCK_MID_PRIMARY, (lock_state & 0xFFFFFFFC));
		if (err != ERROR_OK)
			return err;

		err = spc56x_getlock(bank, LOCK_MID_SECONDARY,  &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc56x_setlock(bank, LOCK_MID_SECONDARY, (lock_state & 0xFFFFFFFC));
		if (err != ERROR_OK)
			return err;
	}

	if (high_mask != 0) {
		err = spc56x_getlock(bank, LOCK_HIGH, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = spc56x_setlock(bank, LOCK_HIGH, (lock_state & 0xFFFFF000));
		if (err != ERROR_OK)
			return err;
	}

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;


	/* Flash erase code */
	if (target_alloc_working_area(target, sizeof(spc56x_flash_erase_code),
			&erase_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, erase_algorithm->address,
			sizeof(spc56x_flash_erase_code), (uint8_t *)spc56x_flash_erase_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, erase_algorithm);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

	/* eraseOption shadowFlag (0x0) */
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

	/* CallBack */
	init_reg_param(&reg_params[5], "r8", 32, PARAM_IN);
	buf_set_u32(reg_params[5].value, 0, 32, 0xFFFFFFFF);


	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[6], "lr", 32, PARAM_IN);
	buf_set_u32(reg_params[6].value, 0, 32, erase_algorithm->address + (sizeof(spc56x_flash_erase_code) - 2));

	init_reg_param(&reg_params[7], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[7].value, 0, 32, target->working_area_phys + target->working_area_size - 1);


	err = target_run_algorithm(target,
			0, NULL,
			8, reg_params,
			erase_algorithm->address, (erase_algorithm->address + erase_algorithm->size),
			10000, &powerpc_info);

	if ((err != ERROR_OK) || (buf_get_u32(reg_params[0].value, 0, 32) != 0)) {
		err = ERROR_TARGET_FAILURE;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);
	destroy_reg_param(&reg_params[7]);

	/* Free resources */
	target_free_working_area(target, ssd_config);
	target_free_working_area(target, erase_algorithm);

	return err;
}

static int spc56x_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

/*offset = base address count = size */
static int spc56x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);
	LOG_DEBUG("%s:%d %s() offset = 0x%08x count = 0x%08x", __FILE__, __LINE__, __func__, offset, count);

	unsigned int i, sector = 0;

	for(i=0; i < bank->num_sectors; i++)
		LOG_DEBUG("-----> bank->sectors[%d].offset = 0x%08x (size = %d K)", i, bank->sectors[i].offset, (bank->sectors[i].size/1024));

	struct powerpc_algorithm powerpc_info;
	struct target *target = bank->target;
	struct spc56x_flash_bank *spc56x_info = bank->driver_priv;

	static struct working_area *source;
	struct working_area *write_algorithm;
	struct working_area *ssd_config;

	struct reg_param reg_params[8];
	SSD_CONFIG *ssd = &spc56x_info->ssd;
	int err = ERROR_OK;


	uint32_t chunk_number;
	uint32_t bytes_remain;
	int32_t tot_sector = 0;
	uint32_t size = 0;
	uint32_t chunk_size = 0x400; /* internal buffer size 1024 bytes */

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

	err = spc56x_setlock(bank, LOCK_LOW_PRIMARY, 0x0);
	if (err != ERROR_OK)
		return err;

	err = spc56x_setlock(bank, LOCK_LOW_SECONDARY, 0x0);
	if (err != ERROR_OK)
		return err;

	err = spc56x_setlock(bank, LOCK_MID_PRIMARY, 0x0);
	if (err != ERROR_OK)
		return err;

	err = spc56x_setlock(bank, LOCK_MID_SECONDARY, 0x0);
	if (err != ERROR_OK)
		return err;

	err = spc56x_setlock(bank, LOCK_HIGH, 0x0);
	if (err != ERROR_OK)
		return err;


	for(i = 0; i < bank->num_sectors; i++)
	{
		if((offset >= bank->sectors[i].offset) && (offset < bank->sectors[i+1].offset)) {
			/* sector found */
			sector = i;
			tot_sector++;
			LOG_DEBUG("Sector found: %d IN_offset= 0x%08x, bank->sectors[%d].offset= 0x%08x, bank->sectors[%d].size= 0x%08x",
						  sector, offset, i, bank->sectors[i].offset, i, bank->sectors[i].size);

			LOG_DEBUG("bank->sectors[%d].size = %d",sector, bank->sectors[sector].size);
			LOG_DEBUG("bank->sectors[%d].offset = 0x%08x", sector, bank->sectors[sector].offset);
			LOG_DEBUG("bank->sectors[%d].is_erased = %d",sector, bank->sectors[sector].is_erased);
			LOG_DEBUG("bank->sectors[%d].is_protected = %d", sector, bank->sectors[sector].is_protected);

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
	if (target_alloc_working_area(target, sizeof(struct _ssd_config),
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
	if (target_alloc_working_area(target, sizeof(spc56x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, write_algorithm->address,
			sizeof(spc56x_flash_write_code), (uint8_t *)spc56x_flash_write_code);
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


	for(i = 0; i < chunk_number; i++)
	{

		err = target_write_buffer(target, source->address,
				size, (uint8_t *)(buffer  + (i * chunk_size)));
		if (err != ERROR_OK) {
			target_free_working_area(target, ssd_config);
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			return err;
		}

		init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* dest */
		init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
		buf_set_u32(reg_params[1].value, 0, 32, bank->base + offset + (i * chunk_size));
		/* buf_set_u32(reg_params[2].value, 0, 32, 0xFC0000); */

		/* size */
		init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
		buf_set_u32(reg_params[2].value, 0, 32, size);

		/* source */
		init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
		buf_set_u32(reg_params[3].value, 0, 32, source->address);

		/* CallBack */
		init_reg_param(&reg_params[4], "r7", 32, PARAM_IN);
		buf_set_u32(reg_params[4].value, 0, 32, 0xFFFFFFFF);



		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[5], "lr", 32, PARAM_IN);	/* lr */
		buf_set_u32(reg_params[5].value, 0, 32, write_algorithm->address +(sizeof(spc56x_flash_write_code) - 2));

		init_reg_param(&reg_params[6], "r1", 32, PARAM_IN);
		buf_set_u32(reg_params[6].value, 0, 32, target->working_area_phys + target->working_area_size);


		err = target_run_algorithm(target,
				0, NULL,
				7, reg_params,
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


		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			goto flash_write_error;
		}
	}

	if(bytes_remain)
	{
		uint8_t padding_bytes;
		uint32_t ind;
		uint8_t * loc_buffer;
		padding_bytes = bytes_remain % C90FL_PAGE_SIZE;

		if (padding_bytes > 0)
		{
			/* allocate new buffer */
			loc_buffer = malloc(bytes_remain + C90FL_PAGE_SIZE - padding_bytes);

			memset(&loc_buffer[bytes_remain], 0xFF, (C90FL_PAGE_SIZE - padding_bytes));

			for (ind = 0; ind < bytes_remain; ind++)
			{
				loc_buffer[ind] = buffer[(chunk_number * chunk_size) + ind];
			}

			bytes_remain += (C90FL_PAGE_SIZE - padding_bytes);

			err = target_write_buffer(target, source->address,
					bytes_remain, (uint8_t *)(loc_buffer));

			free(loc_buffer);
		}
		else
		{
			err = target_write_buffer(target, source->address,
					bytes_remain, (uint8_t *)(buffer  + (chunk_number * chunk_size)));
		}

		if (err != ERROR_OK) {
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			return err;
		}


		init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* dest */
		init_reg_param(&reg_params[1], "r4", 32, PARAM_IN);
		buf_set_u32(reg_params[1].value, 0, 32, bank->base + offset + (chunk_number * chunk_size));
		/* buf_set_u32(reg_params[2].value, 0, 32, 0xFC0000); */

		/* size */
		init_reg_param(&reg_params[2], "r5", 32, PARAM_IN);
		buf_set_u32(reg_params[2].value, 0, 32, bytes_remain);

		/* source */
		init_reg_param(&reg_params[3], "r6", 32, PARAM_IN);
		buf_set_u32(reg_params[3].value, 0, 32, source->address);

		/* CallBack */
		init_reg_param(&reg_params[4], "r7", 32, PARAM_IN);
		buf_set_u32(reg_params[4].value, 0, 32, 0xFFFFFFFF);

		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[5], "lr", 32, PARAM_IN);	/* lr */
		buf_set_u32(reg_params[5].value, 0, 32, write_algorithm->address +(sizeof(spc56x_flash_write_code) - 2));

		init_reg_param(&reg_params[6], "r1", 32, PARAM_IN);
		buf_set_u32(reg_params[6].value, 0, 32, target->working_area_phys + target->working_area_size);


		err = target_run_algorithm(target,
				0, NULL,
				7, reg_params,
				write_algorithm->address, (write_algorithm->address + write_algorithm->size),
				2000000000, &powerpc_info);


		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);
		destroy_reg_param(&reg_params[6]);

		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
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

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);

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

static int spc56x_probe(struct flash_bank *bank)
{
	struct spc56x_flash_bank *spc56x_info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *ssd_config;
	struct working_area *init_algorithm;
	struct reg_param reg_params[3];
	struct powerpc_algorithm powerpc_info;
	SSD_CONFIG *ssd = &spc56x_info->ssd;


	int i;
	int err;
	uint16_t flash_size_in_kb = 0;

	int num_pages;

	num_pages = 0;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	spc56x_info->probed = 0;

	/* The user sets the size manually */
	if (spc56x_info->user_bank_size) {
		LOG_DEBUG("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = spc56x_info->user_bank_size / 1024;
	}

	LOG_INFO("flash: %d kbytes @ 0x%08x", flash_size_in_kb, spc56x_info->user_bank_addr);


	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	/* Default SSD values (keep them in target endianess) */
	uint32_t val;

	if(bank->base == 0x0)
	{
		val = C90FL_C0_REG_BASE;
		ssd->c90flRegBase = fast_target_buffer_get_u32(&val, false);

		val = C0_ARRAY_BASE;
		ssd->mainArrayBase = fast_target_buffer_get_u32(&val, false);

		val = SHADOW_ROW_BASE;
		ssd->shadowRowBase = fast_target_buffer_get_u32(&val, false);

		val = SHADOW_ROW_SIZE;
		ssd->shadowRowSize = fast_target_buffer_get_u32(&val, false);

		val = C90FL_PAGE_SIZE;
		ssd->pageSize = fast_target_buffer_get_u32(&val, false);
	} else if(bank->base == 0x180000)
	{
		val = C90FL_C1_REG_BASE;
		ssd->c90flRegBase = fast_target_buffer_get_u32(&val, false);

		val = C1_ARRAY_BASE;
		ssd->mainArrayBase = fast_target_buffer_get_u32(&val, false);

		val = C90FL_PAGE_SIZE;
		ssd->pageSize = fast_target_buffer_get_u32(&val, false);
	} else if(bank->base == 0x800000)
	{
		val = C90FL_DATA_REG_BASE;
		ssd->c90flRegBase = fast_target_buffer_get_u32(&val, false);

		val = DATA_ARRAY_BASE;
		ssd->mainArrayBase = fast_target_buffer_get_u32(&val, false);

		val = C90FL_PAGE_SIZE_04;
		ssd->pageSize = fast_target_buffer_get_u32(&val, false);
	}

	ssd->lowBlockNum = 0;
	ssd->midBlockNum = 0;
	ssd->highBlockNum = 0;

	ssd->BDMEnable = 0;

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
	{
		target_free_working_area(target, ssd_config);
		return err;
	}


	/* Flash initialization code */
	if (target_alloc_working_area(target, sizeof(spc56x_flash_init_code),
			&init_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash init step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, init_algorithm->address,
			sizeof(spc56x_flash_init_code), (uint8_t *)spc56x_flash_init_code);
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
	buf_set_u32(reg_params[1].value, 0, 32, init_algorithm->address + (sizeof(spc56x_flash_init_code) - 2));
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

	LOG_DEBUG("SDD->c90flRegBase  = 0x%08x", fast_target_buffer_get_u32(&ssd->c90flRegBase, false));
	LOG_DEBUG("SDD->mainArrayBase = 0x%08x", fast_target_buffer_get_u32(&ssd->mainArrayBase, false));
	LOG_DEBUG("SDD->shadowRowBase = 0x%08x", fast_target_buffer_get_u32(&ssd->shadowRowBase, false));
	LOG_DEBUG("SDD->shadowRowSize = 0x%08x", fast_target_buffer_get_u32(&ssd->shadowRowSize, false));

	LOG_DEBUG("SDD->lowBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockNum, false));
	LOG_DEBUG("SDD->midBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockNum, false));
	LOG_DEBUG("SDD->highBlockNum  = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockNum, false));

	LOG_DEBUG("SDD->pageSize   = 0x%08x", fast_target_buffer_get_u32(&ssd->pageSize, false));
	LOG_DEBUG("SDD->BDMEnable     = 0x%u", ssd->BDMEnable);


	/* calculate numbers of pages */
	/* int num_pages = fast_target_buffer_get_u32(&ssd->lowBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n128KBlockNum, false);
	num_pages = num_pages + fast_target_buffer_get_u32(&ssd->midBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n128KBlockNum, false);
	num_pages = num_pages + fast_target_buffer_get_u32(&ssd->highBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n128KBlockNum, false);
	num_pages = num_pages + fast_target_buffer_get_u32(&ssd->nLargeBlockNum, false);
	*/
	if(bank->base == 0x0)
	{
		num_pages = 16;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = spc56x_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 1, 32 * 1024);
		setup_sector(bank, 1, 2, 16 * 1024);
		setup_sector(bank, 3, 2, 32 * 1024);
		setup_sector(bank, 5, 1, 128 * 1024);

		// Mid Flash Blocks
		setup_sector(bank, 6, 2, 128 * 1024);

		// High Flash Blocks
		setup_sector(bank, 8, 8, 128 * 1024);

		spc56x_info->low_max_index = 6;
		spc56x_info->mid_max_index = 8;
		spc56x_info->high_max_index = 16;
		
	} else if(bank->base == 0x180000)
	{
		num_pages = 16;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = spc56x_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 1, 32 * 1024);
		setup_sector(bank, 1, 2, 16 * 1024);
		setup_sector(bank, 3, 2, 32 * 1024);
		setup_sector(bank, 5, 1, 128 * 1024);

		// Mid Flash Blocks
		setup_sector(bank, 6, 2, 128 * 1024);

		// High Flash Blocks
		setup_sector(bank, 8, 8, 128 * 1024);

		spc56x_info->low_max_index = 6;
		spc56x_info->mid_max_index = 8;
		spc56x_info->high_max_index = 16;

		bank->sectors[0].offset = 0x100000;
		bank->sectors[1].offset = 0x108000;
		bank->sectors[2].offset = 0x10C000;
		bank->sectors[3].offset = 0x110000;
		bank->sectors[4].offset = 0x118000;
		bank->sectors[5].offset = 0x120000;
		bank->sectors[6].offset = 0x140000;
		bank->sectors[7].offset = 0x160000;

		bank->sectors[8].offset = 0x0;
		bank->sectors[9].offset = 0x20000;
		bank->sectors[10].offset = 0x40000;
		bank->sectors[11].offset = 0x60000;
		bank->sectors[12].offset = 0x80000;
		bank->sectors[13].offset = 0xA0000;
		bank->sectors[14].offset = 0xC0000;
		bank->sectors[15].offset = 0xE0000;

	} else if(bank->base == 0x800000)
	{
		num_pages = 4;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = spc56x_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);

		spc56x_info->low_max_index = 4;
		spc56x_info->large_max_index = 0;
		spc56x_info->high_max_index = 0;
		spc56x_info->mid_max_index = 0;
	}



	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}


	/* Save flash geometry (in sectors) */
/*
	spc56x_info->low_max_index = fast_target_buffer_get_u32(&ssd->lowBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->lowBlockInfo.n128KBlockNum, false);
	spc56x_info->large_max_index = spc56x_info->low_max_index +  fast_target_buffer_get_u32(&ssd->nLargeBlockNum, false);
	spc56x_info->high_max_index = spc56x_info->large_max_index + fast_target_buffer_get_u32(&ssd->highBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->highBlockInfo.n128KBlockNum, false);
	spc56x_info->mid_max_index = spc56x_info->high_max_index + fast_target_buffer_get_u32(&ssd->midBlockInfo.n16KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n32KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n64KBlockNum, false) + fast_target_buffer_get_u32(&ssd->midBlockInfo.n128KBlockNum, false);
*/



	/* Done */
	spc56x_info->probed = 1;


flash_init_error:
	/* Free resources */
	target_free_working_area(target, ssd_config);
	target_free_working_area(target, init_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return err;
}

static int spc56x_auto_probe(struct flash_bank *bank)
{
	struct spc56x_flash_bank *spc56x_info = bank->driver_priv;

	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	if (spc56x_info->probed)
		return ERROR_OK;
	return spc56x_probe(bank);
}


static int get_spc56x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	/* TODO: retrieve the right info */
	//snprintf(buf, buf_size, "SPC560B - Rev: xx");

	return ERROR_OK;
}


struct flash_driver spc56x_flash = {
	.name = "spc56x",
	.flash_bank_command = spc56x_flash_bank_command,
	.erase = spc56x_erase,
	.protect = spc56x_protect,
	.write = spc56x_write,
	.read = default_flash_read,
	.probe = spc56x_probe,
	.auto_probe = spc56x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = spc56x_protect_check,
	.info = get_spc56x_info,
};

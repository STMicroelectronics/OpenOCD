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
#include <target/aarch64.h>
#include "sr6.h"
#include "../../../contrib/loaders/flash/sr6/sr6.inc"

#define WRTE_OFFSET 0x8000000

struct sr6p7_flash_bank {
	int probed;
	uint32_t user_bank_addr;
	uint32_t user_bank_size;

	uint32_t low_max_index;
	uint32_t mid_max_index;
	uint32_t high_max_index;
	uint32_t large_max_index;

	SSD_CONFIG ssd;
};

/* flash bank s6 <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(sr6p7_flash_bank_command)
{
	struct sr6p7_flash_bank *sr6p7_info;

	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	sr6p7_info = malloc(sizeof(struct sr6p7_flash_bank));
	bank->driver_priv = sr6p7_info;

	sr6p7_info->probed = 0;
	sr6p7_info->user_bank_addr = bank->base;
	sr6p7_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static int sr6p7_protect_check(struct flash_bank *bank)
{
	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	return ERROR_OK;
}

static int sr6p7_setlock(struct flash_bank *bank, uint32_t block_space, uint32_t lock_state)
{
	int err;
	struct target *target = bank->target;
	struct sr6p7_flash_bank *sr6p7_info = bank->driver_priv;
	struct working_area *ssd_config;
	struct working_area *setlock_algorithm;
	struct reg_param reg_params[5];
	struct armv8_algorithm armv8_algorithm_info;
	SSD_CONFIG *ssd = &sr6p7_info->ssd;
	SSD_CONFIG ssd_lock;

	ssd_lock.NVMRegBase = ssd->NVMRegBase;
	ssd_lock.mainArrayBase = ssd->mainArrayBase;
	ssd_lock.lowBlockInfo.n16KBlockNum = ssd->lowBlockInfo.n16KBlockNum;
	ssd_lock.lowBlockInfo.n32KBlockNum = ssd->lowBlockInfo.n32KBlockNum;
	ssd_lock.lowBlockInfo.n64KBlockNum = ssd->lowBlockInfo.n64KBlockNum;

	ssd_lock.midBlockInfo.n16KBlockNum = ssd->midBlockInfo.n16KBlockNum;
	ssd_lock.midBlockInfo.n32KBlockNum = ssd->midBlockInfo.n32KBlockNum;
	ssd_lock.midBlockInfo.n64KBlockNum = ssd->midBlockInfo.n64KBlockNum;

	ssd_lock.highBlockInfo.n16KBlockNum = ssd->highBlockInfo.n16KBlockNum;
	ssd_lock.highBlockInfo.n32KBlockNum = ssd->highBlockInfo.n32KBlockNum;
	ssd_lock.highBlockInfo.n64KBlockNum = ssd->highBlockInfo.n64KBlockNum;

	ssd_lock.n256BlockNum = ssd->n256BlockNum;
	ssd_lock.mainArrayBase = ssd->mainInterfaceFlag;
	ssd_lock.programmableSize = ssd->programmableSize;
	ssd_lock.BDMEnable = ssd->BDMEnable;

	if((bank->base == 0x28000000)||(bank->base == 0x28400000)||(bank->base == 0x29E00000)||(bank->base == 0x29F87000))   //Cluster 0 core 0 || Cluster 0 core 1
	{
		ssd_lock.NVMRegBase = MAIN_REG_BASE_C0;
	}
	else if((bank->base == 0x28800000) || (bank->base == 0x28C00000))   //Cluster 1 core 0 || Cluster 1 core 1
	{
		ssd_lock.NVMRegBase = MAIN_REG_BASE_C1;
	}
	else if((bank->base == 0x29000000) || (bank->base == 0x29400000))  //Cluster 2 core 0 || Cluster 2 core 1
	{
		ssd_lock.NVMRegBase = MAIN_REG_BASE_C2;
	}

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set arch info */
	armv8_algorithm_info.common_magic = ARMV8_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _nvm_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)&ssd_lock);
	if (err != ERROR_OK)
		return err;

	/* Set Lock code */
	if (target_alloc_working_area(target, sizeof(sr6_flash_setlock_code),
			&setlock_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, setlock_algorithm->address,
			sizeof(sr6_flash_setlock_code), (uint8_t *)sr6_flash_setlock_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, setlock_algorithm);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "x0", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

	init_reg_param(&reg_params[1], "x1", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, block_space);

	init_reg_param(&reg_params[2], "x2", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, lock_state);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[3], "x14", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[3].value, 0, 32, (setlock_algorithm->address + (sizeof(sr6_flash_setlock_code) - 2)) | 0x1);

	init_reg_param(&reg_params[4], "sp", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[4].value, 0, 32, target->working_area_phys + target->working_area_size);


	err = target_run_algorithm(target,
			0, NULL,
			5, reg_params,
			setlock_algorithm->address, 0,
			5000, &armv8_algorithm_info);

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

static int sr6p7_getlock(struct flash_bank *bank,
		uint8_t block_space, uint32_t *lock_state)
{
	int err;
	struct target *target = bank->target;
	struct sr6p7_flash_bank *sr6p7_info = bank->driver_priv;
	struct working_area *ssd_config;

	struct working_area *getlock_working_area;

	struct working_area *getlock_algorithm;
	struct reg_param reg_params[5];
	struct armv8_algorithm armv8_algorithm_info;
	SSD_CONFIG *ssd = &sr6p7_info->ssd;
	SSD_CONFIG ssd_lock;

	ssd_lock.NVMRegBase = ssd->NVMRegBase;
	ssd_lock.mainArrayBase = ssd->mainArrayBase;
	ssd_lock.lowBlockInfo.n16KBlockNum = ssd->lowBlockInfo.n16KBlockNum;
	ssd_lock.lowBlockInfo.n32KBlockNum = ssd->lowBlockInfo.n32KBlockNum;
	ssd_lock.lowBlockInfo.n64KBlockNum = ssd->lowBlockInfo.n64KBlockNum;

	ssd_lock.midBlockInfo.n16KBlockNum = ssd->midBlockInfo.n16KBlockNum;
	ssd_lock.midBlockInfo.n32KBlockNum = ssd->midBlockInfo.n32KBlockNum;
	ssd_lock.midBlockInfo.n64KBlockNum = ssd->midBlockInfo.n64KBlockNum;

	ssd_lock.highBlockInfo.n16KBlockNum = ssd->highBlockInfo.n16KBlockNum;
	ssd_lock.highBlockInfo.n32KBlockNum = ssd->highBlockInfo.n32KBlockNum;
	ssd_lock.highBlockInfo.n64KBlockNum = ssd->highBlockInfo.n64KBlockNum;

	ssd_lock.n256BlockNum = ssd->n256BlockNum;
	ssd_lock.mainArrayBase = ssd->mainInterfaceFlag;
	ssd_lock.programmableSize = ssd->programmableSize;
	ssd_lock.BDMEnable = ssd->BDMEnable;

	if((bank->base == 0x28000000)||(bank->base == 0x28400000)||(bank->base == 0x29E00000)||(bank->base == 0x29F87000))   //Cluster 0 core 0 || Cluster 0 core 1
	{
		ssd_lock.NVMRegBase = MAIN_REG_BASE_C0;
	}
	else if((bank->base == 0x28800000) || (bank->base == 0x28C00000))   //Cluster 1 core 0 || Cluster 1 core 1
	{
		ssd_lock.NVMRegBase = MAIN_REG_BASE_C1;
	}
	else if((bank->base == 0x29000000) || (bank->base == 0x29400000))  //Cluster 2 core 0 || Cluster 2 core 1
	{
		ssd_lock.NVMRegBase = MAIN_REG_BASE_C2;
	}

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set arch info */
	armv8_algorithm_info.common_magic = ARMV8_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _nvm_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)&ssd_lock);
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
	if (target_alloc_working_area(target, sizeof(sr6_flash_getlock_code),
			&getlock_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash erase step");
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, getlock_working_area);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, getlock_algorithm->address,
			sizeof(sr6_flash_getlock_code), (uint8_t *)sr6_flash_getlock_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, getlock_algorithm);
		target_free_working_area(target, getlock_working_area);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "x0", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);
	LOG_DEBUG("------> R0= 0x%08x",buf_get_u32(reg_params[0].value, 0, 32));

	init_reg_param(&reg_params[1], "x1", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, block_space);

	init_reg_param(&reg_params[2], "x2", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, getlock_working_area->address);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[3], "x14", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[3].value, 0, 32, (getlock_algorithm->address + (sizeof(sr6_flash_getlock_code) - 2)) | 0x1);

	init_reg_param(&reg_params[4], "sp", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[4].value, 0, 32, target->working_area_phys + target->working_area_size);


	err = target_run_algorithm(target,
			0, NULL,
			5, reg_params,
			getlock_algorithm->address, 0,
			5000, &armv8_algorithm_info);

	if (err != ERROR_OK) {
		err = ERROR_TARGET_FAILURE;
		goto flash_getlock_error;
	}

	err = target_read_buffer(target, getlock_working_area->address,
			4, (uint8_t *)tmp);
	if (err != ERROR_OK) {
		goto flash_getlock_error;
	}


	/*  */
	*lock_state = fast_target_buffer_get_u32(&tmp[0], false);

	LOG_DEBUG("GetLock OK: return value (R0)=0x%08x lock_state=%d",buf_get_u32(reg_params[0].value, 0, 32), *lock_state);
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

static int sr6p7_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	LOG_INFO("%s:%d %s()", __FILE__, __LINE__, __func__);
	LOG_INFO("%s:%d %s() offset = 0x%08x count = 0x%08x", __FILE__, __LINE__, __func__, offset, count);

	unsigned int i, sector = 0;

	uint32_t lock_state;

	for(i=0; i < bank->num_sectors; i++)
		LOG_DEBUG("-----> bank->sectors[%d].offset = 0x%08x (size = %d K)", i, bank->sectors[i].offset, (bank->sectors[i].size/1024));

	struct armv8_algorithm armv8_algorithm_info;
	struct target *target = bank->target;
	struct sr6p7_flash_bank *sr6p7_info = bank->driver_priv;

	static struct working_area *source;
	struct working_area *write_algorithm;
	struct working_area *ssd_config;
	struct working_area *write_working_area_CtxData;
	struct working_area *remote_stack;

	struct reg_param reg_params[6];
	SSD_CONFIG *ssd = &sr6p7_info->ssd;
	int err = ERROR_OK;


	uint32_t chunk_number;
	uint32_t bytes_remain;
	int32_t tot_sector = 0;
	uint32_t size = 0;
	uint32_t chunk_size = 0x400; /* internal buffer size 1024 bytes */

	uint32_t loc_func;

	uint32_t loc_stack[2];

	uint32_t remote_stack_size;

	/* Set arch info */
	armv8_algorithm_info.common_magic = ARMV8_COMMON_MAGIC;

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


	err = sr6p7_getlock(bank, NVM_BLOCK_LOW, &lock_state);
	if (err != ERROR_OK)
		return err;

	err = sr6p7_setlock(bank, NVM_BLOCK_LOW, (lock_state & 0xFFFF0F00));
	if (err != ERROR_OK)
		return err;

	err = sr6p7_getlock(bank, NVM_BLOCK_MID,  &lock_state);
	if (err != ERROR_OK)
		return err;

	err = sr6p7_setlock(bank, NVM_BLOCK_MID, (lock_state & 0xFFFFFF00));
	if (err != ERROR_OK)
		return err;

	err = sr6p7_getlock(bank, NVM_BLOCK_HIGH, &lock_state);
	if (err != ERROR_OK)
		return err;

	err = sr6p7_setlock(bank, NVM_BLOCK_HIGH, (lock_state & 0xFFFFFF00));
	if (err != ERROR_OK)
		return err;
	err = sr6p7_getlock(bank, NVM_BLOCK_256_FIRST, &lock_state);
	if (err != ERROR_OK)
		return err;

	err = sr6p7_setlock(bank, NVM_BLOCK_256_FIRST, (lock_state & 0xF0000000));
	if (err != ERROR_OK)
		return err;

	if (bank->base == 0x29F87000)
	{
		err = sr6p7_getlock(bank, NVM_BLOCK_UTEST,  &lock_state);
		if (err != ERROR_OK)
			return err;
		err = sr6p7_setlock(bank, NVM_BLOCK_UTEST, lock_state & 0xFFFFFFF0);
		if (err != ERROR_OK)
			return err;
	}



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
	if (target_alloc_working_area(target, sizeof(struct _nvm_ssd_config),
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
	if (target_alloc_working_area(target, sizeof(sr6_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	loc_func = fast_target_buffer_get_u32((uint8_t *)&write_algorithm->address, true);

	/* Flash driver uses BLX instruction to call CheckStatus */
	loc_func = loc_func | 1;

	err = target_write_buffer(target, write_algorithm->address,
			sizeof(sr6_flash_write_code), (uint8_t *)sr6_flash_write_code);
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


	err = target_write_buffer(target, write_working_area_CtxData->address + 32,
				32, (uint8_t *)&loc_func);

	remote_stack_size = WRITE_STACK_SIZE + (chunk_size / C55_PROGRAMMABLE_SIZE * 0x70);

   	/* Allocation Remote stack */
	if (target_alloc_working_area_try(target, remote_stack_size, &remote_stack) != ERROR_OK) {
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			target_free_working_area(target, ssd_config);
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
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
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, remote_stack);
			return err;
		}

		init_reg_param(&reg_params[0], "x0", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* factoryPgmFlag  - FALSE to do normal program. */
		init_reg_param(&reg_params[1], "x1", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[1].value, 0, 32, 0);

		/* dest */
		init_reg_param(&reg_params[2], "x2", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[2].value, 0, 32, bank->base + WRTE_OFFSET + offset + (i * chunk_size));

		/* size */
		init_reg_param(&reg_params[3], "x3", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[3].value, 0, 32, size);

		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[4], "x14", 64, PARAM_IN_OUT);	/* lr */
		buf_set_u32(reg_params[4].value, 0, 32, (write_algorithm->address +(sizeof(sr6_flash_write_code) - 2)) | 0x1);


		loc_stack[0] = source->address;
		loc_stack[1] = write_working_area_CtxData->address;

		err = target_write_buffer(target, remote_stack->address + remote_stack_size - 8,
						8, (uint8_t *)&loc_stack);
		if (err != ERROR_OK) {
					target_free_working_area(target, ssd_config);
					target_free_working_area(target, write_algorithm);
					target_free_working_area(target, source);
					target_free_working_area(target, write_working_area_CtxData);
					target_free_working_area(target, remote_stack);
					return err;
		}

		init_reg_param(&reg_params[5], "sp", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[5].value, 0, 32, remote_stack->address +  remote_stack_size - 8);


		err = target_run_algorithm(target,
				0, NULL,
				6, reg_params,
				write_algorithm->address, 0,
				2000000000, &armv8_algorithm_info);

		LOG_DEBUG("Device buffer Size: %d, Number of iteraction: %d, Current iteraction: %d, err: %d", chunk_size, chunk_number, i, err);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);

		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, remote_stack);

			goto flash_write_error;
		}
	}

	if(bytes_remain)
	{

		err = target_write_buffer(target, source->address,
				bytes_remain, (uint8_t *)(buffer  + (i * chunk_size)));
		if (err != ERROR_OK) {
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, remote_stack);
			return err;
		}



		init_reg_param(&reg_params[0], "x0", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

		/* factoryPgmFlag  - FALSE to do normal program. */
		init_reg_param(&reg_params[1], "x1", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[1].value, 0, 32, 0);

		/* dest */
		init_reg_param(&reg_params[2], "x2", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[2].value, 0, 32, bank->base + WRTE_OFFSET + offset + (i * chunk_size));


		/* size */
		init_reg_param(&reg_params[3], "x3", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[3].value, 0, 32, bytes_remain);

		/*
		 * Link register (in).
		 * Set link register to the breakpoint instruction at the end of the buffer.
		 * We use a software breakpoint to notify when done with algorithm execution.
		 */
		init_reg_param(&reg_params[4], "x14", 64, PARAM_IN_OUT);	/* lr */
		buf_set_u32(reg_params[4].value, 0, 32, (write_algorithm->address +(sizeof(sr6_flash_write_code) - 2)) | 0x1);

		loc_stack[0] = source->address;
		loc_stack[1] = write_working_area_CtxData->address;

		err = target_write_buffer(target, remote_stack->address +  remote_stack_size - 8 ,
						8, (uint8_t *)&loc_stack);
		if (err != ERROR_OK) {
					target_free_working_area(target, ssd_config);
					target_free_working_area(target, write_algorithm);
					target_free_working_area(target, source);
					target_free_working_area(target, write_working_area_CtxData);
					target_free_working_area(target, remote_stack);
					return err;
		}

		init_reg_param(&reg_params[5], "sp", 64, PARAM_IN_OUT);
		buf_set_u32(reg_params[5].value, 0, 32, remote_stack->address +  remote_stack_size - 8);

		err = target_run_algorithm(target,
				0, NULL,
				6, reg_params,
				write_algorithm->address, 0,
				2000000000, &armv8_algorithm_info);

		LOG_DEBUG("Device buffer Size: %d, Number of iteraction: %d, Current iteraction: %d, err: %d", chunk_size, chunk_number, i, err);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		destroy_reg_param(&reg_params[5]);



		if (err != ERROR_OK)  {
			err = ERROR_TARGET_FAILURE;
			target_free_working_area(target, write_algorithm);
			target_free_working_area(target, source);
			target_free_working_area(target, write_working_area_CtxData);
			target_free_working_area(target, remote_stack);

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
	target_free_working_area(target, remote_stack);

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

static int sr6p7_writeToErase(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	LOG_INFO("%s:%d %s()", __FILE__, __LINE__, __func__);
	LOG_INFO("%s:%d %s() offset = 0x%08x count = 0x%08x", __FILE__, __LINE__, __func__, offset, count);

	unsigned int i, sector = 0;

	for(i=0; i < bank->num_sectors; i++)
		LOG_DEBUG("-----> bank->sectors[%d].offset = 0x%08x (size = %d K)", i, bank->sectors[i].offset, (bank->sectors[i].size/1024));

	struct armv8_algorithm armv8_algorithm_info;
	struct target *target = bank->target;
	struct sr6p7_flash_bank *sr6p7_info = bank->driver_priv;

	struct working_area *erase_algorithm;
	struct working_area *ssd_config;

	struct reg_param reg_params[6];
	SSD_CONFIG *ssd = &sr6p7_info->ssd;
	int err = ERROR_OK;

	int32_t tot_sector = 0;

	/* Set arch info */
	armv8_algorithm_info.common_magic = ARMV8_COMMON_MAGIC;

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
	if (target_alloc_working_area(target, sizeof(struct _nvm_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
		return err;

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(sr6_flash_erase_code),
			&erase_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	err = target_write_buffer(target, erase_algorithm->address,
			sizeof(sr6_flash_write_code), (uint8_t *)sr6_flash_erase_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, erase_algorithm);
		return err;
	}

	init_reg_param(&reg_params[0], "x0", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

	/* factoryPgmFlag  - FALSE to do normal program. */
	init_reg_param(&reg_params[1], "x1", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, 0);

	/* dest */
	init_reg_param(&reg_params[2], "x2", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, bank->base + WRTE_OFFSET + offset);

	/* size */
	init_reg_param(&reg_params[3], "x3", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[3].value, 0, 32, count);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[4], "x14", 64, PARAM_IN_OUT);	/* lr */
	buf_set_u32(reg_params[4].value, 0, 32, (erase_algorithm->address +(sizeof(sr6_flash_erase_code) - 2)) | 0x1);

	/*
		* Stack pointer.
	*/
	init_reg_param(&reg_params[5], "sp", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[5].value, 0, 32, target->working_area_phys + target->working_area_size);

	err = target_run_algorithm(target,
			0, NULL,
			6, reg_params,
			erase_algorithm->address, 0,
			2000000000, &armv8_algorithm_info);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

	if (err != ERROR_OK)  {
		err = ERROR_TARGET_FAILURE;
		target_free_working_area(target, erase_algorithm);

		goto flash_erase_error;
	}


	err = target_read_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);

	if (err != ERROR_OK)
	{
		err = ERROR_TARGET_FAILURE;
	}

	target_free_working_area(target, erase_algorithm);

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


static int sr6p7_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{

	unsigned int i;
	int err;
	struct target *target = bank->target;
	struct sr6p7_flash_bank *sr6p7_info = bank->driver_priv;

	LOG_INFO("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t low_mask = 0;
	uint32_t mid_mask = 0;
	uint32_t high_mask = 0;
	uint32_t utest_mask = 0;
	N256_BLOCK_SEL nLargeBlockSelect;
	//uint32_t loc_stack[3];
	nLargeBlockSelect.first256BlockSelect = 0;
	nLargeBlockSelect.second256BlockSelect = 0;

	uint8_t buffer[0x40000];  //256KB

	memset(buffer, 0xFF, sizeof(buffer));

	if (bank->base == 0x28000000)
	{
		for (i = first; i <= last; i++) {
			if (i < sr6p7_info->low_max_index) {
				low_mask |= (1 <<  (i + 1));
			} else if (i < sr6p7_info->large_max_index) {
				nLargeBlockSelect.first256BlockSelect |= (1 << (i - sr6p7_info->low_max_index));
			} else if (i < sr6p7_info->mid_max_index) {
				mid_mask |= (1 << (i - sr6p7_info->low_max_index));
			} else if (i < sr6p7_info->high_max_index) {
				high_mask |= (1 << (i - sr6p7_info->mid_max_index));
			}
		}
	}
	else if (bank->base == 0x28400000)
	{
		for (i = first; i <= last; i++) {
			if (i < sr6p7_info->mid_max_index) {
				mid_mask |= (1 << i);
			} else if (i < sr6p7_info->large_max_index) {
				nLargeBlockSelect.first256BlockSelect |= (1 << ((i - sr6p7_info->mid_max_index)+14));
			} else if (i < sr6p7_info->low_max_index) {
				low_mask |= (1 << i);
			} else if (i < sr6p7_info->high_max_index) {
				high_mask |= (1 << i);
			}
		}
	}
	else if (bank->base == 0x28800000)
	{
		for (i = first; i <= last; i++) {
			if (i < sr6p7_info->low_max_index) {
				low_mask |= (1 << (i + 1));
			} else if (i < sr6p7_info->large_max_index) {
				if (i<15)
				{
					nLargeBlockSelect.first256BlockSelect |= (1 << (i - sr6p7_info->low_max_index));
				}
				else
				{
					nLargeBlockSelect.first256BlockSelect |= (1 << (i - sr6p7_info->low_max_index + 7));
				}
			} else if (i < sr6p7_info->mid_max_index) {
				mid_mask |= (1 << i);
			} else if (i < sr6p7_info->high_max_index) {
				high_mask |= (1 << i);
			}
		}
	}
	else if (bank->base == 0x28C00000)
	{
		for (i = first; i <= last; i++) {
			if (i < sr6p7_info->mid_max_index) {
				mid_mask |= (1 << i);
			} else if (i < sr6p7_info->large_max_index) {
				if (1<15)
				{
					nLargeBlockSelect.first256BlockSelect |= (1 << ((i - sr6p7_info->mid_max_index)+7));
				}
				else
				{
					nLargeBlockSelect.first256BlockSelect |= (1 << ((i - sr6p7_info->mid_max_index)+15));
				}
			} else if (i < sr6p7_info->low_max_index) {
				low_mask |= (1 << i);
			} else if (i < sr6p7_info->high_max_index) {
				high_mask |= (1 << i);
			}
		}
	}
	else if (bank->base == 0x29000000)
	{
		for (i = first; i <= last; i++) {
			if (i < sr6p7_info->low_max_index) {
				low_mask |= (1 << (i + 17));
			} else if (i < sr6p7_info->large_max_index) {
				nLargeBlockSelect.first256BlockSelect |= (1 << (i - sr6p7_info->low_max_index));
			} else if (i < sr6p7_info->mid_max_index) {
				mid_mask |= (1 << i);
			} else if (i < sr6p7_info->high_max_index) {
				high_mask |= (1 << i);
			}
		}
	}
	else if (bank->base == 0x29400000)
	{
		for (i = first; i <= last; i++) {
			if (i < sr6p7_info->mid_max_index) {
				mid_mask |= (1 << i);
			} else if (i < sr6p7_info->large_max_index) {
				nLargeBlockSelect.first256BlockSelect |= (1 << ((i - sr6p7_info->mid_max_index) + 7));
			} else if (i < sr6p7_info->low_max_index) {
				low_mask |= (1 << i);
			} else if (i < sr6p7_info->high_max_index) {
				high_mask |= (1 << i);
			}
		}
	}
	else if (bank->base == 0x29E00000)
	{
		for (i = first; i <= last; i++) {
			if (i < sr6p7_info->high_max_index) {
				high_mask |= (1 << i);
			} else if (i < sr6p7_info->large_max_index) {
				nLargeBlockSelect.first256BlockSelect |= (1 << ((i - sr6p7_info->mid_max_index)));
			} else if (i < sr6p7_info->low_max_index) {
				low_mask |= (1 << i);
			} else if (i < sr6p7_info->mid_max_index) {
				mid_mask |= (1 << i);
			}
		}
	}
	else if (bank->base == 0x29F87000)
	{
		utest_mask = 1;
	}



	/* unlock flash registers */
	uint32_t lock_state;

	if (low_mask != 0) {
		err = sr6p7_getlock(bank, NVM_BLOCK_LOW, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = sr6p7_setlock(bank, NVM_BLOCK_LOW, (lock_state & 0xFFFF0F00));
		if (err != ERROR_OK)
			return err;
	}

	if (mid_mask != 0) {
		err = sr6p7_getlock(bank, NVM_BLOCK_MID,  &lock_state);
		if (err != ERROR_OK)
			return err;

		err = sr6p7_setlock(bank, NVM_BLOCK_MID, (lock_state & 0xFFFFFF00));
		if (err != ERROR_OK)
			return err;
	}

	if (high_mask != 0) {
		err = sr6p7_getlock(bank, NVM_BLOCK_HIGH, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = sr6p7_setlock(bank, NVM_BLOCK_HIGH, (lock_state & 0xFFFFFF00));
		if (err != ERROR_OK)
			return err;
	}

	if (nLargeBlockSelect.first256BlockSelect != 0) {
		err = sr6p7_getlock(bank, NVM_BLOCK_256_FIRST, &lock_state);
		if (err != ERROR_OK)
			return err;

		err = sr6p7_setlock(bank, NVM_BLOCK_256_FIRST, (lock_state & 0xF0000000));
		if (err != ERROR_OK)
			return err;
	}

	if (utest_mask !=0)
	{
		err = sr6p7_getlock(bank, NVM_BLOCK_UTEST,  &lock_state);
		if (err != ERROR_OK)
			return err;
		err = sr6p7_setlock(bank, NVM_BLOCK_UTEST, lock_state & 0xFFFFFFF0);
		if (err != ERROR_OK)
			return err;
	}

	for (i=first; i<= last; i++)
	{
		sr6p7_writeToErase(bank, buffer, bank->sectors[i].offset, bank->sectors[i].size);
	}

	return err;
}


static int sr6p7_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
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

static int sr6p7_probe(struct flash_bank *bank)
{
	struct sr6p7_flash_bank *sr6p7_info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *ssd_config;
	struct working_area *init_algorithm;
	struct reg_param reg_params[3];
	struct armv8_algorithm armv8_algorithm_info;
	SSD_CONFIG *ssd = &sr6p7_info->ssd;


	int i;
	int err;
	uint16_t flash_size_in_kb = 0;

	int num_pages;

	num_pages = 0;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	sr6p7_info->probed = 0;

	/* The user sets the size manually */
	if (sr6p7_info->user_bank_size) {
		LOG_DEBUG("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = sr6p7_info->user_bank_size / 1024;
	}

	LOG_INFO("flash: %d kbytes @ 0x%08x", flash_size_in_kb, sr6p7_info->user_bank_addr);


	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* Set arch info */
	armv8_algorithm_info.common_magic = ARMV8_COMMON_MAGIC;

	/* SSD structure */
	if (target_alloc_working_area(target, sizeof(struct _nvm_ssd_config),
			&ssd_config) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do SSD config allocation");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};


	/* Default SSD values (keep them in target endianess) */
	uint32_t val;
	val = MAIN_REG_BASE_C0;
    ssd->NVMRegBase = fast_target_buffer_get_u32(&val, true);

    val = MAIN_ARRAY_BASE;
	ssd->mainArrayBase = fast_target_buffer_get_u32(&val, true);

	val = C55_PROGRAMMABLE_SIZE;
	ssd->programmableSize = fast_target_buffer_get_u32(&val, true);


	ssd->lowBlockInfo.n16KBlockNum = 0;
	ssd->lowBlockInfo.n32KBlockNum = 0;
	ssd->lowBlockInfo.n64KBlockNum = 0;

	ssd->midBlockInfo.n16KBlockNum = 0;
	ssd->midBlockInfo.n32KBlockNum = 0;
	ssd->midBlockInfo.n64KBlockNum = 0;

	ssd->highBlockInfo.n16KBlockNum = 0;
	ssd->highBlockInfo.n32KBlockNum = 0;
	ssd->highBlockInfo.n64KBlockNum = 0;

	ssd->n256BlockNum = 0;

	ssd->mainInterfaceFlag = 1;

	ssd->BDMEnable = 1;

	err = target_write_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK)
	{
		target_free_working_area(target, ssd_config);
		return err;
	}


	/* Flash initialization code */
	if (target_alloc_working_area(target, sizeof(sr6_flash_init_code),
			&init_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash init step");
		target_free_working_area(target, ssd_config);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	err = target_write_buffer(target, init_algorithm->address,
			sizeof(sr6_flash_init_code), (uint8_t *)sr6_flash_init_code);
	if (err != ERROR_OK) {
		target_free_working_area(target, ssd_config);
		target_free_working_area(target, init_algorithm);
		return err;
	}

	/* ssd_config (in), return value (out) */
	init_reg_param(&reg_params[0], "x0", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ssd_config->address);

	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[1], "x14", 64, PARAM_IN_OUT);

	//bit 0 is set to 1 because bx lr needs to have bit 0 set to 1
	buf_set_u32(reg_params[1].value, 0, 32, (init_algorithm->address + (sizeof(sr6_flash_init_code) - 2)) | 0x1);

	/*
	* Stack Pointer (in).
	*/
	init_reg_param(&reg_params[2], "sp", 64, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, target->working_area_phys + target->working_area_size);

	err = target_run_algorithm(target,
			0, NULL,
			3, reg_params,
			init_algorithm->address, 0,
			5000, &armv8_algorithm_info);


	//if ((err != ERROR_OK) || (buf_get_u32(reg_params[0].value, 0, 32) != 0)) {
	if (err != ERROR_OK) {
		err = ERROR_TARGET_FAILURE;
		goto flash_init_error;
	}

	err = target_read_buffer(target, ssd_config->address,
			sizeof(SSD_CONFIG), (uint8_t *)ssd);
	if (err != ERROR_OK) {
		goto flash_init_error;
	}

	LOG_DEBUG("SDD->NVMRegBase  = 0x%08x", fast_target_buffer_get_u32(&ssd->NVMRegBase, true));
	LOG_DEBUG("SDD->mainArrayBase = 0x%08x", fast_target_buffer_get_u32(&ssd->mainArrayBase, true));

	LOG_DEBUG("SDD->lowBlockInfo->n16KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockInfo.n16KBlockNum, true));
	LOG_DEBUG("SDD->lowBlockInfo->n32KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockInfo.n32KBlockNum, true));
	LOG_DEBUG("SDD->lowBlockInfo->n64KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->lowBlockInfo.n64KBlockNum, true));

	LOG_DEBUG("SDD->midBlockInfo->n16KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockInfo.n16KBlockNum, true));
	LOG_DEBUG("SDD->midBlockInfo->n32KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockInfo.n32KBlockNum, true));
	LOG_DEBUG("SDD->midBlockInfo->n64KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->midBlockInfo.n64KBlockNum, true));

	LOG_DEBUG("SDD->highBlockInfo->n16KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockInfo.n16KBlockNum, true));
	LOG_DEBUG("SDD->highBlockInfo->n32KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockInfo.n32KBlockNum, true));
	LOG_DEBUG("SDD->highBlockInfo->n64KBlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->highBlockInfo.n64KBlockNum, true));


	LOG_DEBUG("SDD->n256BlockNum   = 0x%08x", fast_target_buffer_get_u32(&ssd->n256BlockNum, true));

	LOG_DEBUG("SDD->mainInterfaceFlag  = 0x%u", ssd->mainInterfaceFlag);
	LOG_DEBUG("SDD->programmableSize   = 0x%08x", fast_target_buffer_get_u32(&ssd->programmableSize, true));
	LOG_DEBUG("SDD->BDMEnable     = 0x%u", ssd->BDMEnable);


	if(bank->base == 0x28000000)   //Cluster 0 core 0
	{
		num_pages = 22;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);
		setup_sector(bank, 4, 2, 32 * 1024);
		setup_sector(bank, 6, 2, 64 * 1024);

		// Large Flash Blocks
		setup_sector(bank, 8, 14, 256 * 1024);

		sr6p7_info->low_max_index = 8;
		sr6p7_info->large_max_index = 22;
		sr6p7_info->high_max_index = 0;
		sr6p7_info->mid_max_index = 0;
	}
	if(bank->base == 0x28400000)   //Cluster 0 core 1
	{
		num_pages = 22;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);
		setup_sector(bank, 4, 2, 32 * 1024);
		setup_sector(bank, 6, 2, 64 * 1024);

		// Large Flash Blocks
		setup_sector(bank, 8, 14, 256 * 1024);

		sr6p7_info->low_max_index = 0;
		sr6p7_info->large_max_index = 22;
		sr6p7_info->high_max_index = 0;
		sr6p7_info->mid_max_index = 8;
	}
	if(bank->base == 0x28800000)   //Cluster 1 core 0
	{
		num_pages = 23;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);
		setup_sector(bank, 4, 2, 32 * 1024);
		setup_sector(bank, 6, 2, 64 * 1024);

		// Large Flash Blocks
		setup_sector(bank, 8, 15, 256 * 1024);

		sr6p7_info->low_max_index = 8;
		sr6p7_info->large_max_index = 23;
		sr6p7_info->high_max_index = 0;
		sr6p7_info->mid_max_index = 0;
	}
	if(bank->base == 0x28C00000)   //Cluster 1 core 1
	{
		num_pages = 23;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);
		setup_sector(bank, 4, 2, 32 * 1024);
		setup_sector(bank, 6, 2, 64 * 1024);

		// Large Flash Blocks
		setup_sector(bank, 8, 15, 256 * 1024);

		sr6p7_info->low_max_index = 0;
		sr6p7_info->mid_max_index = 8;
		sr6p7_info->large_max_index = 23;
		sr6p7_info->high_max_index = 0;

	}
	if(bank->base == 0x29000000)   //Cluster 2 core 0
	{
		num_pages = 15;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Low Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);
		setup_sector(bank, 4, 2, 32 * 1024);
		setup_sector(bank, 6, 2, 64 * 1024);

		// Large Flash Blocks
		setup_sector(bank, 8, 7, 256 * 1024);

		sr6p7_info->low_max_index = 8;
		sr6p7_info->large_max_index = 15;
		sr6p7_info->mid_max_index = 0;
		sr6p7_info->high_max_index = 0;

	}
	if(bank->base == 0x29400000)   //Cluster 2 core 1
	{
		num_pages = 15;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// Mid Flash Blocks
		setup_sector(bank, 0, 4, 16 * 1024);
		setup_sector(bank, 4, 2, 32 * 1024);
		setup_sector(bank, 6, 2, 64 * 1024);

		// Large Flash Blocks
		setup_sector(bank, 8, 7, 256 * 1024);

		sr6p7_info->low_max_index = 0;
		sr6p7_info->mid_max_index = 8;
		sr6p7_info->large_max_index = 15;
		sr6p7_info->high_max_index = 0;
	}
	else if(bank->base == 0x29E00000) //Cluster 0 Core 0 Data Flash
	{
		num_pages = 8;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// High Flash Blocks
		setup_sector(bank, 0, 8, 64 * 1024);

		sr6p7_info->low_max_index = 0;
		sr6p7_info->large_max_index = 0;
		sr6p7_info->high_max_index = 8;
		sr6p7_info->mid_max_index = 0;
	}
	else if(bank->base == 0x29F87000) //Cluster 0 Core 0 UTEST For Boot Record
	{
		num_pages = 1;

		/* check that calculation result makes sense */
		assert(num_pages > 0);

		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		bank->base = sr6p7_info->user_bank_addr;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		bank->size = 0;

		// High Flash Blocks
		setup_sector(bank, 0, 1, 4 * 1024);

		sr6p7_info->low_max_index = 1;
		sr6p7_info->large_max_index = 0;
		sr6p7_info->high_max_index = 0;
		sr6p7_info->mid_max_index = 0;
	}

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	/* Done */
	sr6p7_info->probed = 1;


flash_init_error:
	/* Free resources */
	target_free_working_area(target, ssd_config);
	target_free_working_area(target, init_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return err;
}

static int sr6p7_auto_probe(struct flash_bank *bank)
{
	struct sr6p7_flash_bank *sr6p7_info = bank->driver_priv;

	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	if (sr6p7_info->probed)
		return ERROR_OK;
	return sr6p7_probe(bank);
}


static int get_sr6p7_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	LOG_DEBUG("%s:%d %s()",
		__FILE__, __LINE__, __func__);

	/* TODO: retrieve the right info */
	//snprintf(buf, buf_size, "SPC560B - Rev: xx");

	return ERROR_OK;
}


struct flash_driver sr6p7_flash = {
	.name = "sr6p7",
	.flash_bank_command = sr6p7_flash_bank_command,
	.erase = sr6p7_erase,
	.protect = sr6p7_protect,
	.write = sr6p7_write,
	.read = default_flash_read,
	.probe = sr6p7_probe,
	.auto_probe = sr6p7_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = sr6p7_protect_check,
	.info = get_sr6p7_info,
};

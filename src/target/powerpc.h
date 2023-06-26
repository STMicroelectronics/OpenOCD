/****************************************************************************
 * Copyright (C) 2016  STMicroelectronics - All Rights Reserved             *
 *                                                                          *
 * May be copied or modified under the terms of the LGPL v2.1.              *
 *                                                                          *
 * ST makes no warranty express or implied including but not limited to,    *
 * any warranty of                                                          *
 *                                                                          *
 * (i)  merchantability or fitness for a particular purpose and/or          *
 * (ii) requirements, for a particular purpose in relation to the LICENSED  *
 *      MATERIALS, which is provided AS IS, WITH ALL FAULTS. ST does not    *
 *      represent or warrant that the LICENSED MATERIALS provided here      *
 *      under is free of infringement of any third party patents,           *
 *      copyrights, trade secrets or other intellectual property rights.    *
 *      ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE        *
 *      EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                     *
 ****************************************************************************/

#ifndef POWERPC_H
#define POWERPC_H

#include <jtag/jtag.h>

#define POWERPC_COMMON_MAGIC 0x1AE43041

#define POWERPC_NUM_REGS        38
#define POWERPC_NUM_CORE_REGS   38

/* Debug context status */
#define DEBUG_RUNNING       0x01
#define DEBUG_HALT          0x02
#define DEBUG_STEP          0x04
#define DEBUG_STEP_TWICE    0x08
#define DEBUG_SOFT_BREAK    0x10

struct powerpc_algorithm {
	int common_magic;
	uint32_t context[POWERPC_NUM_REGS];
};

struct mcu_jtag {
	struct jtag_tap *tap;
};

/* Keep CPU status (as per CPU scan register - see OnCE CPUSCR ) */
struct cpu_status {
	uint32_t CTL;           /* Control State Register         */
	uint32_t IR;            /* Instruction Register           */
	uint32_t PC;            /* Program Counter Register       */
	uint32_t MSR;           /* Machine State Register         */
	uint32_t WBBR_HIGH;     /* Write-Back Bus Register (high) */
	uint32_t WBBR_LOW;      /* Write-Back Bus Register (low)  */
};

struct dbg_watchpoint {
	int32_t  avail; /* total number of available hardware watchpoints */
	int32_t  free;  /* free hardware watchpoints */
	uint32_t mask;  /* bit mask to track used hardware watchpoints */
};

struct hw_breakpoint {
	int32_t  avail; /* total number of available hardware breakpoints */
	int32_t  free;  /* free hardware breakpoints */
	uint32_t mask;  /* bit mask to track used hardware breakpoints */
};

struct powerpc_common {
	struct mcu_jtag    jtag_info;
	struct cpu_status  cpu_info;
	struct reg_cache  *core_cache;

	/* Shortcut to PC register */
	struct reg        *pc;

	uint32_t core_regs[POWERPC_NUM_CORE_REGS];

	/* Context information */
	uint32_t ctx_info;

	/* Debug watchpoints */
	struct dbg_watchpoint dwp;

	/* Hardware breakpoints */
	struct hw_breakpoint hwb;

	/* Register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned int num);
	int (*write_core_reg)(struct target *target, unsigned int num);

	int common_magic;
};

struct powerpc_core_reg {
	uint32_t num;
	struct target *target;
	struct powerpc_common *powerpc_common;
};

static inline
struct powerpc_common *target_to_powerpc(struct target *target)
{
	return target->arch_info;
}

extern const struct command_registration powerpc_command_handlers[];
/*
static int powerpc_write_memory_noturn(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer);
*/
#endif /* POWERPC_H */

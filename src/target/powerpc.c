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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "breakpoints.h"
#include "target_type.h"
#include "helper/types.h"
#include "helper/jep106.h"
#include "jtag/interface.h"
#include "target_request.h"
#include "algorithm.h"
#include "register.h"
#include "powerpc.h"
#include "semihosting_common.h"
#include "../../contrib/loaders/flash/powerpc/spc58x.inc"



/*
 * GDB registers
 * based on gdb-7.8.2/gdb/features/power-core.xml
 */
static const struct {
	unsigned id;
	const char *name;
	enum reg_type type;
	const char *group;
	const char *feature;
	int flag;
} powerpc_regs[] = {
	{  0,  "r0", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  1,  "r1", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  2,  "r2", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  3,  "r3", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  4,  "r4", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  5,  "r5", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  6,  "r6", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  7,  "r7", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  8,  "r8", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{  9,  "r9", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 10, "r10", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 11, "r11", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 12, "r12", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 13, "r13", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 14, "r14", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 15, "r15", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 16, "r16", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 17, "r17", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 18, "r18", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 19, "r19", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 20, "r20", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 21, "r21", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 22, "r22", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 23, "r23", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 24, "r24", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 25, "r25", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 26, "r26", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 27, "r27", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 28, "r28", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 29, "r29", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 30, "r30", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },
	{ 31, "r31", REG_TYPE_INT32, NULL, "org.gnu.gdb.power.core", 0 },

	{ 32, "pc",  REG_TYPE_CODE_PTR, NULL, "org.gnu.gdb.power.core", 0 },
	{ 33, "msr", REG_TYPE_INT32,    NULL, "org.gnu.gdb.power.core", 0 },
	{ 34, "cr",  REG_TYPE_INT32,    NULL, "org.gnu.gdb.power.core", 0 },
	{ 35, "lr",  REG_TYPE_CODE_PTR, NULL, "org.gnu.gdb.power.core", 0 },
	{ 36, "ctr", REG_TYPE_INT32,    NULL, "org.gnu.gdb.power.core", 0 },
	{ 37, "xer", REG_TYPE_INT32,    NULL, "org.gnu.gdb.power.core", 0 },
};


static int powerpc_save_context(struct target *target);
static int powerpc_restore_context(struct target *target);
static int powerpc_set_watchpoint(struct target *target, struct watchpoint *watchpoint);
static int powerpc_set_breakpoint(struct target *target, struct breakpoint *breakpoint);
static int powerpc_once_spr_read(struct target *target, uint32_t reg_id, uint32_t *val);
static int powerpc_once_spr_write(struct target *target, uint32_t reg_id, uint32_t val);
static int powerpc_assert_reset(struct target *target);
static int powerpc_unset_breakpoint(struct target *target, struct breakpoint *breakpoint);
static void powerpc_once_get_idcode(struct target *target);
static int powerpc_nexus_mem_read(struct target *target, target_addr_t address,	uint32_t size, uint32_t count, uint32_t *buffer);
static int powerpc_once_reg_write(struct jtag_tap *tap, int flush, uint8_t reg, uint32_t * data);
static int powerpc_once_target_status(struct jtag_tap *tap);

/* Instruction Address Compare (breakpoint) */
#define DBSR_IAC1_EVENT         0x00800000
#define DBSR_IAC2_EVENT         0x00400000
#define DBSR_IAC3_EVENT         0x00200000
#define DBSR_IAC4_EVENT         0x00100000

/* Data Address Compare (watchpoint) */
#define DBSR_DAC1_READ_EVENT    0x00080000
#define DBSR_DAC1_WRITE_EVENT   0x00040000
#define DBSR_DAC2_READ_EVENT    0x00020000
#define DBSR_DAC2_WRITE_EVENT   0x00010000

#define DBSR_IAC_MASK           0x00F00000
#define DBSR_DAC_MASK           0x000F0000

/**************************************************************************/
/* OnCE implementation                                                    */
/**************************************************************************/

/* e200z0 OnCE Register Addressing */

#define E200Zxx_ONCE_JTAG_ID            0x02
#define E200Zxx_ONCE_CPUSCR             0x10
#define E200Zxx_ONCE_NONE               0x11
#define E200Zxx_ONCE_OCR                0x12

#define E200Zxx_ONCE_DAC3               0x18
#define E200Zxx_ONCE_DAC4               0x19

#define E200Zxx_ONCE_IAC1               0x20
#define E200Zxx_ONCE_IAC2               0x21
#define E200Zxx_ONCE_IAC3               0x22
#define E200Zxx_ONCE_IAC4               0x23

#define E200Zxx_ONCE_DAC1               0x24
#define E200Zxx_ONCE_DAC2               0x25

#define E200Zxx_ONCE_DVC1               0x26
#define E200Zxx_ONCE_DVC2               0x27

#define E200Zxx_ONCE_IAC5               0x28
#define E200Zxx_ONCE_IAC6               0x29
#define E200Zxx_ONCE_IAC7               0x2A
#define E200Zxx_ONCE_IAC8               0x2B

#define E200Zxx_ONCE_DDEAR              0x2C
#define E200Zxx_ONCE_EDDEAR             0x2D
#define E200Zxx_ONCE_EDBCR0             0x2E
#define E200Zxx_ONCE_EDBSR0             0x2F

#define E200Zxx_ONCE_DBSR               0x30
#define E200Zxx_ONCE_DBCR0              0x31
#define E200Zxx_ONCE_DBCR1              0x32
#define E200Zxx_ONCE_DBCR2              0x33

#define E200Zxx_ONCE_DBCR4              0x35
#define E200Zxx_ONCE_DBCR5              0x36
#define E200Zxx_ONCE_DBCR6              0x37
#define E200Zxx_ONCE_DBCR7              0x38
#define E200Zxx_ONCE_DBCR8              0x39

#define E200Zxx_ONCE_EDBSRMSK0          0x3C
#define E200Zxx_ONCE_DDAM               0x3D
#define E200Zxx_ONCE_DEVENT             0x3E
#define E200Zxx_ONCE_EDBRAC0            0x3F
#define E200Zxx_ONCE_DBL1CCSR0          0x40

#define E200Zxx_ONCE_MPU0CSR0           0x6D
#define E200Zxx_ONCE_PERF_MON           0x6E

#define E200Zxx_ONCE_GPR0               0x70
#define E200Zxx_ONCE_GPR1               0x71
#define E200Zxx_ONCE_GPR2               0x72
#define E200Zxx_ONCE_GPR3               0x73
#define E200Zxx_ONCE_GPR4               0x74
#define E200Zxx_ONCE_GPR5               0x75
#define E200Zxx_ONCE_GPR6               0x76
#define E200Zxx_ONCE_GPR7               0x77
#define E200Zxx_ONCE_GPR8               0x78
#define E200Zxx_ONCE_GPR9               0x79

#define E200Zxx_ONCE_CDACNTL            0x7A
#define E200Zxx_ONCE_CDADATA            0x7B

#define E200Zxx_ONCE_NEXUS3             0x7C

#define E200Zxx_ONCE_LSLR               0x7D

#define E200Zxx_ONCE_ENABLE             0x7E
#define E200Zxx_ONCE_BYPASS             0x7F

/* OnCE status register */
#define ONCE_STATUS_MCKL              0x0200
#define ONCE_STATUS_ERR               0x0100
#define ONCE_STATUS_RESET             0x0040
#define ONCE_STATUS_HALT              0x0020
#define ONCE_STATUS_STOP              0x0010
#define ONCE_STATUS_DEBUG             0x0008
#define ONCE_STATUS_WAIT              0x0004

/* OnCE current status */
#define ONCE_IS_DEFAULT		(ONCE_STATUS_MCKL | 0x0001)
#define ONCE_IS_HALT		(ONCE_STATUS_HALT | ONCE_STATUS_STOP)
#define ONCE_IS_ERROR		(ONCE_STATUS_ERR)
#define ONCE_IS_DEBUG		(ONCE_STATUS_DEBUG)
#define ONCE_IS_RESET		(ONCE_STATUS_RESET)

/* JTAG Controller */
#define JTAG_INSTR_IDCODE               0x01
#define JTAG_INSTR_SAMPLE_PRELOAD       0x02
#define JTAG_INSTR_SAMPLE               0x03
#define JTAG_INSTR_EXTEST               0x04

/* SPC584B - Chorus2M */
#define JTAG_INSTR_ENABLE_JTAG_PASSWORD 0x07
#define JTAG_INSTR_HIGHZ                0x09
#define JTAG_INSTR_CLAMP                0x0C
#define JTAG_INSTR_ENABLE_DCI_CR        0x0E
#define JTAG_INSTR_ACCESS_AUX_TAP_TCU   0x20  /* or 0x3E */
#define JTAG_INSTR_ACCESS_AUX_NPC_PD    0x21
#define JTAG_INSTR_ACCESS_AUX_JDC       0x26
#define JTAG_INSTR_ACCESS_AUX_HSM       0x27
#define JTAG_INSTR_ACCESS_AUX_CORE_2    0x2A
#define JTAG_INSTR_ACCESS_AUX_BUS_MON_0 0x34
#define JTAG_INSTR_ACCESS_AUX_BUS_MON_1 0x35
#define JTAG_INSTR_ACCESS_AUX_BUS_MON_2 0x36
#define JTAG_INSTR_BYPASS               0x3F

/* Bolero ?
#define JTAG_INSTR_ACCESS_AUX_TAP_TCU   0x10
#define JTAG_INSTR_ACCESS_AUX_TAP_ONCE  0x11
#define JTAG_INSTR_ACCESS_AUX_TAP_NPC   0x12
#define JTAG_INSTR_BYPASS               0x1F
*/

#define POWERPC_NUM_GP_REGS 32

/* Useful PowerPC opcodes */

#define POWERPC_E_ORI_OPCODE 0x06

#define POWERPC_MFSPR_OPCODE ((0x1F << 26) | 0x02A6)
#define POWERPC_MTSPR_OPCODE ((0x1F << 26) | 0x03A6)

#define  POWERPC_MTCR_OPCODE   ((0x1F << 26) | 0x0FF120)
#define  POWERPC_MFCR_OPCODE   ((0x1F << 26) | 0x000026)

/* Load from memory */
#define POWERPC_LBZ_OPCODE   0x22
#define POWERPC_E_LBZ_OPCODE 0x0C
#define POWERPC_LHZ_OPCODE   0x28
#define POWERPC_E_LHZ_OPCODE 0x16
#define POWERPC_LWZ_OPCODE   0x20
#define POWERPC_E_LWZ_OPCODE 0x14

#if 0 // multiple word
#define POWERPC_E_LMW_OPCODE 0x06  // need to understand D8
#define POWERPC_LMW_OPCODE   0x2E
#endif

#define POWERPC_BKPT_INST    0x00000000

/* Store to memory */
#define POWERPC_STB_OPCODE    0x26
#define POWERPC_E_STB_OPCODE  0x0D
#define POWERPC_STH_OPCODE    0x2C
#define POWERPC_E_STH_OPCODE  0x17
#define POWERPC_STW_OPCODE    0x24
#define POWERPC_E_STW_OPCODE  0x15

#if 0 // multiple word
#define POWERPC_E_STMW_OPCODE 0x06  // need to understand D8
#define POWERPC_STMW_OPCODE   0x2F
#endif

#define POWERPC_NOP 0x60000000  /* Could be also E_NOP (e_ori 0,0,0) --> 0x18000000 */

#define IRSTAT8_MASK 0x00000002
#define IS_VLE( _op) (((_op) & IRSTAT8_MASK) >> 0x1)

#define BUILD_INSTR(_op, _rS, _rA, _imm) (((_op) << 26) | ((_rS) << 21) | ((_rA) << 16) | (_imm))

#define BUILD_SPR_INSTR(_op, _rD, _rA) ((_op) | ((_rD) << 21) | ((_rA & 0x01F) << 16) | (((_rA & 0x3E0) >> 5) << 11))

#define BUILD_CR_INSTR(_op, _rD) ((_op) | (_rD << 21))

/* SPRs register */
#define SPR_XER 1
#define SPR_LR  8
#define SPR_CTR 9

#define SPR_DBCR0 308
#define SPR_DBCR1 309
#define SPR_DBCR2 310
#define SPR_DBCR3 561

#define SPR_DBSR  304

#define SPR_IAC1  312
#define SPR_IAC2  313
#define SPR_IAC3  314
#define SPR_IAC4  315

#define SPR_DAC1  316
#define SPR_DAC2  317

/* PC adjustment */
#define PC_NO_ERR_MASK     0x0
#define PC_MIN_0x04_MASK   0x1
#define PC_MIN_0x08_MASK   0x2
#define PC_MIN_0x0C_MASK   0x3
#define PC_MIN_0x10_MASK   0x4
#define PC_MIN_0x14_MASK   0x5

#define CTL_PCOFST_MASK        0x0000F000
#define CTL_IRSTATUS_BIT_MASK  0xFFFFFC00
#define CTL_IRSTATUS2_BIT_MASK 0xFFFFFF0F
#define CTL_PCOFST(_val) (((_val) &  CTL_PCOFST_MASK) >> 12)

/* MSR bit mask */
#define MSR_UCLE (1 << 26)
#define MSR_WE   (1 << 18)
#define MSR_CE   (1 << 17)
#define MSR_EE   (1 << 15)
#define MSR_PR   (1 << 14)
#define MSR_FP   (1 << 13)
#define MSR_ME   (1 << 12)
#define MSR_DE   (1 <<  9)
#define MSR_IS   (1 <<  5)
#define MSR_DS   (1 <<  4)
#define MSR_RI   (1 <<  2)

enum {
	GPR0,
	GPR1,
	GPR2,
	GPR3,
	GPR4,
	GPR5,
	GPR6,
	GPR7,
	GPR8,
	GPR9,
	GPR10,
	GPR11,
	GPR12,
	GPR13,
	GPR14,
	GPR15,
	GPR16,
	GPR17,
	GPR18,
	GPR19,
	GPR20,
	GPR21,
	GPR22,
	GPR23,
	GPR24,
	GPR25,
	GPR26,
	GPR27,
	GPR28,
	GPR29,
	GPR30,
	GPR31,

	PC,
	MSR,
	CR,
	LR,
	CTR,
	XER
};

/* Use R31 as scratch */
#define GPR_SCRATCH   GPR31

/* JTAG decoding */

#define EXTRACT_MFG(X)        (((X) & 0x00000ffe) >> 1)
#define EXTRACT_PART(X)       (((X) & 0x0ffff000) >> 12)
#define EXTRACT_SUBFAMILY(X)  (((X) & 0x0001E000) >> 13)
#define EXTRACT_APPFAMILY(X)  (((X) & 0x00070000) >> 17)
#define EXTRACT_TECHNOLOGY(X) (((X) & 0x00300000) >> 20)
#define EXTRACT_DESIGN(X)     (((X) & 0x00C00000) >> 22)
#define EXTRACT_VER_MAJOR(X)  (((X) & 0x0f000000) >> 24)
#define EXTRACT_VER_MINOR(X)  (((X) & 0xf0000000) >> 28)
#define EXTRACT_VER(X)        (((X) & 0xf0000000) >> 28)

#define JEP106_STMICROELECTRONICS  0x020

#define POWERPC_40nm_MAX_DEVICES   10
static char *devices_40nm[POWERPC_40nm_MAX_DEVICES] =
{
		"SPC582B - (Chorus1M)",
		"SPC584C / SPC58EC - (Chorus4M)",
		"SPC584B - (Chorus2M)",
		"SPC58NH / SPC58EH - (Chorus 10M)",
		"unknown",
		"unknown",
		"unknown",
		"unknown",
		"SPC58xG / SPC58xE - (Chorus/Eiger 6M)",
		"SPC58xN - (Bernina 6M)"
};

/* Technology */
#define POWERPC_TECH_40nm   1

static char *technology_name[] =
{
		"55nm",   /* Velvety, Sphaero */
		"40nm",
		"90nm",
		"55nm",   /* Lavaredo, K2 */
};

static void powerpc_decode_idcode(uint32_t idcode)
{
	uint32_t manufacturer;

	manufacturer = EXTRACT_MFG(idcode);

	if (manufacturer == JEP106_STMICROELECTRONICS) {
		uint32_t subfamily = EXTRACT_SUBFAMILY(idcode);
		uint32_t app_family = EXTRACT_APPFAMILY(idcode);
		uint32_t tech = EXTRACT_TECHNOLOGY(idcode);
		uint32_t design = EXTRACT_DESIGN(idcode);
		uint32_t ver_major = EXTRACT_VER_MAJOR(idcode);
		uint32_t ver_minor = EXTRACT_VER_MINOR(idcode);

		char *device_name = NULL;

		if (tech == POWERPC_TECH_40nm) {
			if (subfamily < POWERPC_40nm_MAX_DEVICES) {
				device_name = devices_40nm[subfamily];
			}
		}
		LOG_INFO("0x%08x\n"
				"\tmfg: 0x%3.3x (%s),\n"
				"\tdevice: 0x%2.2x - %s,\n"
				"\tapplication: %1.1u,\n"
				"\ttechnology: %s,\n"
				"\tdesign: %1.1u,\n"
				"\tcut: %1.1u.%1.1u",
				(unsigned int)idcode,
				(unsigned int)manufacturer,
				jep106_manufacturer(manufacturer),
				(unsigned int)subfamily,
				((device_name == NULL) ? "unknown" : device_name),
				(unsigned int)app_family,
				technology_name[tech],
				(unsigned int)design,
				(unsigned int)ver_major + 1,
				(unsigned int)ver_minor);
	} else {

		LOG_INFO("0x%08x "
				"(mfg: 0x%3.3x (%s), part: 0x%4.4x, ver: 0x%1.1x)",
				(unsigned int)idcode,
				(unsigned int)EXTRACT_MFG(idcode),
				jep106_manufacturer(EXTRACT_MFG(idcode)),
				(unsigned int)EXTRACT_PART(idcode),
				(unsigned int)EXTRACT_VER(idcode));
	}
}


static int powerpc_verify_pointer(struct command_invocation *cmd, struct powerpc_common *powerpc)
{
	if (powerpc->common_magic != POWERPC_COMMON_MAGIC) {
		command_print(cmd, "target is not a Powerpc");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/*
 * PowerPC interface on top on JTAG API's interface to send JTAG commands
 */
static int powerpc_jtag_ir(struct jtag_tap *tap, uint8_t * ir_in, uint8_t * ir_out, int ir_len)
{
	jtag_add_plain_ir_scan(ir_len, ir_out, ir_in, TAP_IDLE);
	return ERROR_OK;
}

static int powerpc_jtag_ir_u8(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out, int ir_len)
{
	return powerpc_jtag_ir(tap, ir_in, &ir_out, ir_len);
}

static int powerpc_jtag_ir_u32(struct jtag_tap *tap, uint32_t * ir_in, uint32_t ir_out, int ir_len)
{
	return powerpc_jtag_ir(tap, (uint8_t *) ir_in, (uint8_t *) &ir_out, ir_len);
}

/*
 * Send JTAG command instructions
 */
static
int powerpc_jtag_send_ir(struct jtag_tap *tap, uint8_t cmd_out)
{
	return powerpc_jtag_ir_u8(tap, NULL, cmd_out, tap->ir_length);
}

/*
 * OnCE single word instruction
 */
static int powerpc_once_ir_exec(struct jtag_tap *tap, int flush, uint32_t instr, uint32_t rw, uint32_t go, uint32_t ex)
{
	int err;
	uint32_t ir_out;
	uint32_t ir_in;

	ir_out = instr | (ex << 7) | (go << 8) | (rw << 9);
	err = powerpc_jtag_ir_u32(tap, &ir_in, ir_out, 10);
	if (err != ERROR_OK)
		return err;

	if (flush)
		err = jtag_execute_queue();

	return err;
}

/*
 * Enable OnCE TAP controller
 */
static int powerpc_once_gain_access(struct jtag_tap *tap)
{
	/* Load opcode into the JTAGC Instruction Register */
	return powerpc_jtag_send_ir(tap, JTAG_INSTR_ACCESS_AUX_CORE_2);
}

/*
 * Release OnCE TAP controller
 */
static int powerpc_once_release(struct target *target)
{
	int err;
	uint32_t dr_in = 0;

	err = powerpc_once_gain_access(target->tap);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NONE, 0, 0, 0);
	if (err != ERROR_OK)
		return err;

	/*
	 * This sequence here is to get the TAP scan chain to go through the TAP_DREXIT2 state
	 * OnCE is released only when a _DR scan is going to the TAP_DREXIT2 state.
	 */
	jtag_add_plain_dr_scan(32, (uint8_t *) &dr_in, (uint8_t *) NULL, TAP_DRPAUSE);
	jtag_add_statemove(TAP_IDLE);

	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

/*
 * Get IDCODE for JTAGC main TAP controller.
 */ 
static uint32_t powerpc_jtag_get_idcode(struct target *target)
{
	int dr_in, dr_out, err;

	/* Make sure to leave OnCE ownership to access the main JTAGC controller */
   err = powerpc_once_release(target);
   if (err != ERROR_OK)
      return err;

	/* Get the JTAGC IDCODE */
	dr_in = dr_out = 0;
	powerpc_jtag_send_ir(target->tap, JTAG_INSTR_IDCODE);
	jtag_add_plain_dr_scan(32, (uint8_t *) &dr_out, (uint8_t *) &dr_in, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	/* Restore the OnCE debug mode */
	if (target->state == TARGET_HALTED)
		powerpc_once_gain_access(target->tap);

	LOG_DEBUG("JTAG IDCODE : 0x%" PRIx32, dr_in);
	return dr_in;
}

static int powerpc_once_read_chain_regs(struct jtag_tap *tap, uint32_t * data, int len)
{
	int err;
	int i;
	uint32_t dr_out;

	for (i = 0; i < len; i++) {

		err = powerpc_once_gain_access(tap);
		if (err != ERROR_OK)
			return err;

		err = powerpc_once_ir_exec(tap, 1, E200Zxx_ONCE_CPUSCR, 1 /*rw*/, 0 /*go*/, 0 /*ex*/);
		if (err != ERROR_OK)
			return err;

		dr_out = 0;
		jtag_add_plain_dr_scan(32, (uint8_t *) &dr_out, (uint8_t *) &data[i], TAP_IDLE);

		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;
	}

	return err;
}

static int powerpc_once_write_chain_regs(struct jtag_tap *tap, uint32_t * data, int len)
{
	int err;
	int i;

	for (i = 0; i < len; i++) {

		err = powerpc_once_gain_access(tap);
		if (err != ERROR_OK)
			return err;

		err = powerpc_once_ir_exec(tap, 1, E200Zxx_ONCE_CPUSCR, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
		if (err != ERROR_OK)
			return err;

		jtag_add_plain_dr_scan(32, (uint8_t *) &data[i], (uint8_t *) NULL, TAP_IDLE);

		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;
	}

	return err;
}

#if 0
static int powerpc_nexus_mem_write(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int err;

	uint32_t data_out;
	uint32_t data_in;


			err = powerpc_once_gain_access(target->tap);
		if (err != ERROR_OK)
			return err;

		err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NEXUS3, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
		if (err != ERROR_OK)
			return err;

		/* Select RWA as Nexus register 0x9 */
		 buf_set_u32((uint8_t *)&data_out, 0, 32, 0x9);

		 data_out = (data_out<<1) | 0x1;

		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);

		/* Write RWA */
		buf_set_u32((uint8_t *)&data_out, 0, 32, address);
		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		/* Select RWCS 0x7 */
		buf_set_u32((uint8_t *)&data_out, 0, 32, 0x7);

		data_out = (data_out<<1) | 0x1;

		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		/* Wrtite RWCS*/
		data_out = (0x1<<31 | 0x1<<30 | 0x2 << 27 | 0x3 << 22 | 0x1 << 2) & 0x3C600004;
		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		/* Select RWD */
		buf_set_u32((uint8_t *)&data_out, 0, 32, 0xA);

		data_out = (data_out<<1) | 0x1;

		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		/* Write RWD */
		data_out = buffer[0] << 24 | buffer[1] << 16 | buffer[2] << 8 | buffer[3];
		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		/* Read back RWCS so check ERR and DV bits */
		/* Select RWCS 0x7 */
		buf_set_u32((uint8_t *)&data_out, 0, 32, 0x7);

		data_out = data_out<<1;

		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		/* Read RWCS*/
		data_out = 0x0;
		jtag_add_plain_dr_scan(8, (uint8_t *) &data_out, (uint8_t *) &data_in, TAP_IDLE);
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		if (data_in & 0x3)
		{
			/* Write doesn't complete*/
			return err;
		}
	return err;
}

#endif


static int powerpc_nexus_mem_read(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint32_t *buffer)
{
	int err;

	uint32_t data_out;
	uint32_t data_in;
	uint32_t data_in_RWCS;
	uint8_t reg_out;
	uint8_t loc_size;
	uint32_t i, j;
	uint8_t loc_count;

	uint8_t *loc_point_b;
	uint16_t *loc_point_h;

	loc_point_b = (uint8_t *)buffer;
	loc_point_h = (uint16_t *)buffer;

	*buffer =0x0;

	err = ERROR_OK;

	err = powerpc_once_gain_access(target->tap);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NEXUS3, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Select RDA 0x9 */
	reg_out = (0x9 << 0x1) | 0x1;
	jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	/* Write RDA */
	data_out = (uint32_t)address;
	jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NEXUS3, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Select RWCS 0x7 */
	reg_out = (0x7 << 1) | 0x1;
	jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	/* Wrtite RWCS*/
	switch (size) {
	case 8:
		loc_size = 0x3;
		loc_count = 2;
		break;
	case 4:
		loc_size = 0x2;
		loc_count = 1;
		break;
	case 2:
		loc_size = 0x1;
		loc_count = 1;
		break;
	case 1:
		loc_size = 0x0;
		loc_count = 1;
		break;
	default:
		loc_size = 0x0;
		loc_count = 1;
	}
	data_out = (0x1<<31 | loc_size << 27 | 0x3 << 22 | count << 2);
	jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NEXUS3, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	for (j = 0; j < count; j++)
	{
		for (i = 0; i < loc_count; i ++)
		{
			/* Select RWD */
			reg_out = 0xA << 1;
			jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
			err = jtag_execute_queue();
			if (err != ERROR_OK)
				return err;

			/* Read RWD */
			data_out = 0x0;
			data_in = 0x0;
			jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) &data_in, TAP_IDLE);
			err = jtag_execute_queue();
			if (err != ERROR_OK)
				return err;

			switch (size) {
			case 8:
			case 4:
				*buffer = data_in;
				buffer++;
				break;
			case 2:
				*loc_point_h = (uint16_t) data_in & 0xFFFF;
				loc_point_h++;
				break;
			case 1:
				*loc_point_b = (uint8_t) data_in & 0xFF;
				loc_point_b++;
				break;
			}
		}
	}

	/* Read back RWCS so check ERR and DV bits */
	/* Select RWCS 0x7 */
	reg_out = 0x7 << 1;
	jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	/* Read RWCS*/
	data_out = 0x0;
	data_in_RWCS = 0x0;
	jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) &data_in_RWCS, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	if (data_in_RWCS & 0x2)
	{
		/* Write doesn't complete*/
		err = ERROR_FAIL;
	}
/*
	if ((data_in_RWCS & 0x80000000) == 0x0)
	{
		// End of reading
		end_read = 1;
	}
*/
	return err;
}


static int powerpc_nexus_mem_write(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint32_t *buffer)
{
	int err;

	uint32_t data_out;
	uint32_t data_in_RWCS;
	uint8_t reg_out;
	uint8_t loc_size;
	uint32_t i, j;
	uint8_t loc_count;

	uint8_t *loc_point_b;
	uint16_t *loc_point_h;
	uint32_t error_count;

	loc_point_b = (uint8_t *)buffer;
	loc_point_h = (uint16_t *)buffer;

	err = ERROR_OK;

	err = powerpc_once_gain_access(target->tap);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NEXUS3, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Select RDA 0x9 */
	reg_out = (0x9 << 0x1) | 0x1;
	jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	/* Write RDA */
	data_out = (uint32_t)address;
	jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NEXUS3, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Select RWCS 0x7 */
	reg_out = (0x7 << 1) | 0x1;
	jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	/* Write RWCS*/
	switch (size) {
	case 8:
		loc_size = 0x3;
		loc_count = 2;
		break;
	case 4:
		loc_size = 0x2;
		loc_count = 1;
		break;
	case 2:
		loc_size = 0x1;
		loc_count = 1;
		break;
	case 1:
		loc_size = 0x0;
		loc_count = 1;
		break;
	default:
		loc_size = 0x0;
		loc_count = 1;
	}
	data_out = (0x1<<31 | 0x1<<30 | loc_size << 27 | 0x3 << 22 | count << 2);
	jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(target->tap, 1, E200Zxx_ONCE_NEXUS3, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	for (j = 0; j < count; j++)
	{
		for (i = 0; i < loc_count; i ++)
		{
			/* Select RWD */
			reg_out = (0xA << 1) | 0x1;
			jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
			err = jtag_execute_queue();
			if (err != ERROR_OK)
				return err;

			/* Write RWD */

			switch (size) {
			case 8:
			case 4:
				data_out = *buffer;
				buffer++;
				break;
			case 2:
				data_out = (uint32_t)(*loc_point_h  & 0xFFFF);
				loc_point_h++;
				break;
			case 1:
				data_out = (uint32_t)(*loc_point_b & 0xFF);
				loc_point_b++;
				break;
			}
			jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) NULL, TAP_IDLE);
			err = jtag_execute_queue();
			if (err != ERROR_OK)
				return err;
		}

		error_count = 0x0;
		data_out = 0x0;
		data_in_RWCS = 0x1;

		while (data_in_RWCS & 0x1)
		{
			error_count++;

			if (error_count > 1000)
				return err;

			/* Read back RWCS so check ERR and DV bits */
			/* Select RWCS 0x7 */
			reg_out = 0x7 << 1;
			jtag_add_plain_dr_scan(8, (uint8_t *) &reg_out, (uint8_t *) NULL, TAP_IDLE);
			err = jtag_execute_queue();
			if (err != ERROR_OK)
				return err;

			/* Read RWCS*/
			jtag_add_plain_dr_scan(32, (uint8_t *) &data_out, (uint8_t *) &data_in_RWCS, TAP_IDLE);
			err = jtag_execute_queue();
			if (err != ERROR_OK)
				return err;

			if (data_in_RWCS & 0x2)
			{
				/* Error during access*/
				err = ERROR_FAIL;
			}
		}

	}
/*
	if ((data_in_RWCS & 0x80000000) == 0x0)
	{
		// End of reading
		end_read = 1;
	}
*/
	return err;
}



/** OnCE read register */
static int powerpc_once_reg_read(struct jtag_tap *tap, int flush, uint8_t reg, uint32_t * data)
{
	int err;
	uint32_t dr_in = 0;

	/* Gain access to OnCE TAP */
	err = powerpc_once_gain_access(tap);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(tap, flush, reg, 1 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	jtag_add_plain_dr_scan(32, (uint8_t *) &dr_in, (uint8_t *) data, TAP_IDLE);

	if (flush)
		err = jtag_execute_queue();

	return err;
}

/*
 * Get IDCODE for OnCE auxiliary TAP controller.
 */ 
static void powerpc_once_get_idcode(struct target *target)
{
		uint32_t dr_out = 0;

		/* May be usefully in the future to distinguish auxiliary TAPs */
		powerpc_once_reg_read(target->tap, 1, E200Zxx_ONCE_JTAG_ID, &dr_out);

		LOG_INFO("OnCE IDCODE = 0x%08x", dr_out);
}

/** OnCE write register */
static int powerpc_once_reg_write(struct jtag_tap *tap, int flush, uint8_t reg, uint32_t * data)
{
	int err;
	uint32_t dr_in;
	uint32_t dr_out;

	/* Gain access to OnCE TAP */
	err = powerpc_once_gain_access(tap);
	if (err != ERROR_OK)
		return err;

	/* Execute the command */
	err = powerpc_once_ir_exec(tap, flush, reg, 0 /*rw*/, 0 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	dr_out = *data;
	dr_in = 0;
	jtag_add_plain_dr_scan(32, (uint8_t *) &dr_out, (uint8_t *) &dr_in, TAP_IDLE);

	if (flush)
		err = jtag_execute_queue();

	return err;
}

/* OnCE status register (OSR) read */
static int powerpc_once_read_status(struct jtag_tap *tap, uint32_t *data)
{
	int err;
	uint32_t ir_out;
	uint32_t ir_in;

	/* Gain access to OnCE TAP */
	err = powerpc_once_gain_access(tap);
	if (err != ERROR_OK)
		return err;

	ir_out = E200Zxx_ONCE_NONE;
	ir_in = 0;
	err = powerpc_jtag_ir_u32(tap, &ir_in, ir_out, 10);
	if (err != ERROR_OK)
		return err;

	err = jtag_execute_queue();

	*data = ir_in;

	return err;
}

/* OnCE CPU Status registers (CPUSCR) read */
static int powerpc_once_read_cpu_scan_regs(struct jtag_tap *tap, struct cpu_status *cpu_info)
{
	int err;
	uint32_t data[6];

	err = powerpc_once_read_chain_regs(tap, data, 6);
	if (err == ERROR_OK) {
		cpu_info->WBBR_LOW  = data[0];
		cpu_info->WBBR_HIGH = data[1];
		cpu_info->MSR       = data[2];
		cpu_info->PC        = data[3];
		cpu_info->IR        = data[4];
		cpu_info->CTL       = data[5];
	}

	return err;
}

/* OnCE CPU Status registers (CPUSCR) write */
static int powerpc_once_write_cpu_scan_regs(struct jtag_tap *tap, struct cpu_status *cpu_info)
{
	uint32_t data[6];

	data[0] = cpu_info->WBBR_LOW;
	data[1] = cpu_info->WBBR_HIGH;
	data[2] = cpu_info->MSR;
	data[3] = cpu_info->PC;
	data[4] = cpu_info->IR;
	data[5] = cpu_info->CTL;

	return powerpc_once_write_chain_regs(tap, data, 6);
}

#if 0
static int powerpc_once_print_cpu_scan_regs(struct target *target)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);

	LOG_DEBUG(">>>>>> Enter %s()", __func__);

	LOG_INFO("CPUSCR - WWBRLow  = 0x%08x",powerpc->cpu_info.WBBR_LOW);
	LOG_INFO("CPUSCR - WWBRHigh = 0x%08x",powerpc->cpu_info.WBBR_HIGH);
	LOG_INFO("CPUSCR - MSR      = 0x%08x",powerpc->cpu_info.MSR);
	LOG_INFO("CPUSCR - PC       = 0x%08x",powerpc->cpu_info.PC);
	LOG_INFO("CPUSCR - IR       = 0x%08x",powerpc->cpu_info.IR);
	LOG_INFO("CPUSCR - CTL      = 0x%08x",powerpc->cpu_info.CTL);

	return ERROR_OK;
}
#endif
/*
 * This function manage the different CPUSCR registers before exit form a debug session
 *  1) Reset the CTL.IR Status bits
 *  2) Adjust the PC following the CTL.PCOFST value
 *  3) IR register
 */
static int powerpc_once_reset_cpu_scan_regs(struct target *target)
{
	uint32_t ctl_pcofst;
	uint32_t word_size = 4;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	ctl_pcofst = CTL_PCOFST(powerpc->cpu_info.CTL);

	/*
	 * If CTL[PCOFST] is not zero needs a PC adjustment
	 * and IR set with a nop as described in AN4035 3.7.2
	 *
	 * CTL[PCOFST]:
	 * 0000 - No correction
	 * 0001 - PC = PC - 0x04
	 * 0010 - PC = PC - 0x08
	 * 0011 - PC = PC - 0x0C
	 * 0100 - PC = PC - 0x10
	 * 0101 - PC = PC - 0x14
	 *
	 */
	if (ctl_pcofst) {
		powerpc->cpu_info.PC = powerpc->cpu_info.PC - (ctl_pcofst * word_size);
		powerpc->cpu_info.IR = POWERPC_NOP;

		/*
		 * IR-Status Bits reset to 0s
		 * as described in RM0044 8.4.8 pag. 193
		 */
		powerpc->cpu_info.CTL = powerpc->cpu_info.CTL & CTL_IRSTATUS_BIT_MASK;
	} else {
		/* Reset only the CTL.IR-Status bit [2..5] */
		powerpc->cpu_info.CTL = powerpc->cpu_info.CTL & CTL_IRSTATUS2_BIT_MASK;
	}

	return ERROR_OK;
}


/* Exec the instruction set in IR loaded by a CPUSCR command */
static int powerpc_once_exec(struct jtag_tap *tap, uint32_t go, uint32_t ex)
{
	int err;
	uint32_t dr_in = 0;

	err = powerpc_once_gain_access(tap);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_ir_exec(tap, 1, E200Zxx_ONCE_NONE, 0 /*rw*/, go, ex);
	if (err != ERROR_OK)
		return err;

	jtag_add_plain_dr_scan(32, (uint8_t *) &dr_in, (uint8_t *) NULL, TAP_IDLE);

	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	return err;
}

/* Access (read) a CPU general purpose register (GPR) */
static int powerpc_once_gpr_read(struct target *target, uint32_t reg_id, uint32_t *val)
{
	int err;
	struct cpu_status cpu_info;

	memset(&cpu_info, 0, sizeof(cpu_info));

	/* GPR id is in the range [0,31] */
	reg_id &= 0x1F;

	/*
	 * Executing
	 *
	 *    ORI Rx,Rx,0
	 *
	 *    Contents of Rx is latched into WBBR_LOW
	 */
	cpu_info.IR = BUILD_INSTR(POWERPC_E_ORI_OPCODE, reg_id, reg_id, 0xD000);
	cpu_info.CTL = (0x80000000 >> 30);
	cpu_info.PC = 0xF7FB8000;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Read CPU info */
	err = powerpc_once_read_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	*val = cpu_info.WBBR_LOW;




	return ERROR_OK;
}

/* Access (write) a CPU general purpose register (GPR) */
static int powerpc_once_gpr_write(struct target *target, uint32_t reg_id, uint32_t val)
{
	int err;
	struct cpu_status cpu_info;

	memset(&cpu_info, 0, sizeof(cpu_info));

	/* GPR id is in the range [0,31] */
	reg_id &= 0x1F;

	/*
	 * Setting CTL[FFRA] causes WBBR_LOW to be
	 * used as source register in ORI instruction:
	 *
	 *    ORI Rx, WBBR_LOW, 0
	 *
	 */
	cpu_info.IR = BUILD_INSTR(POWERPC_E_ORI_OPCODE, reg_id, reg_id, 0xD000);

	cpu_info.CTL = (0x80000000 >> 30) | (0x80000000 >> 21); /* Set also FFRA */
	cpu_info.WBBR_LOW = val;
	cpu_info.PC = 0xF7FB8000;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int powerpc_once_spr_read(struct target *target, uint32_t reg_id, uint32_t *val)
{
	int err;
	struct cpu_status cpu_info;
	uint32_t value1;

	memset(&cpu_info, 0, sizeof(cpu_info));

	/* Save GPR31 register value */
	err = powerpc_once_gpr_read(target, GPR_SCRATCH, &value1);
	if (err != ERROR_OK)
		return err;

	/*
	 * Executing
	 *
	 *    mfspr R31, reg_id
	 *
	 *    Contents of Rx is latched into WBBR_LOW
	 */
	cpu_info.IR = BUILD_SPR_INSTR(POWERPC_MFSPR_OPCODE, GPR_SCRATCH, reg_id);
	cpu_info.CTL = (0x80000000 >> 30);
	cpu_info.PC = 0xF7FB8000;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Read CPU info */
	err = powerpc_once_read_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	*val = cpu_info.WBBR_LOW;

	/* Restore GPR31 */
	err = powerpc_once_gpr_write(target, GPR_SCRATCH, value1);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int powerpc_once_spr_write(struct target *target, uint32_t reg_id, uint32_t val)
{
	int err;
	struct cpu_status cpu_info;

	/*
	 * From OnCE Emulation documentation (DM00046345), cap 3.11
	 * DBCR0-3 (308-310 and 561), DBSR (304), DBCTN (??), IAC1-4 (312-315), DAC1-2 (316-317)
	 * cannot be written by single stepping over mtspr like the other SPRs
	 * while in external debug mode.
	 */
	if (((reg_id >= SPR_IAC1) && (reg_id <= SPR_DAC2)) ||
	    ((reg_id >= SPR_DBCR0) && (reg_id <= SPR_DBCR2)) ||
	     (reg_id == SPR_DBSR) ||
	     (reg_id == SPR_DBCR3))
		return ERROR_FAIL;

	memset(&cpu_info, 0, sizeof(cpu_info));

    /* mtspr reg_id, WBBR.low */
	cpu_info.IR = BUILD_SPR_INSTR(POWERPC_MTSPR_OPCODE, GPR_SCRATCH, reg_id);
	cpu_info.CTL = (0x80000000 >> 30) | (0x80000000 >> 21); /* Set also FFRA */
	cpu_info.WBBR_LOW = val;
	cpu_info.PC = 0xF7FB8000;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

/* Function read the Condition Register (CR) */
static int powerpc_once_cr_read(struct target *target, uint32_t *val)
{
	int err;
	struct cpu_status cpu_info;

	memset(&cpu_info, 0, sizeof(cpu_info));

	/*
	 * Executing
	 *
	 *    mfcr rD
	 *
	 *    Contents of Rx is latched into WBBR_LOW
	 */
	cpu_info.IR = BUILD_CR_INSTR(POWERPC_MFCR_OPCODE, GPR_SCRATCH);
	cpu_info.CTL = (0x80000000 >> 30);
	cpu_info.PC = 0xF7FB8000;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Read CPU info */
	err = powerpc_once_read_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	*val = cpu_info.WBBR_LOW;

	return ERROR_OK;
}

static int powerpc_once_cr_write(struct target *target, uint32_t val)
{
	int err;
	struct cpu_status cpu_info;

	memset(&cpu_info, 0, sizeof(cpu_info));

    /*
     * mtcr rS  WBBR.low
     */
	cpu_info.IR = BUILD_CR_INSTR(POWERPC_MTCR_OPCODE, GPR_SCRATCH);

	cpu_info.CTL = (0x80000000 >> 30) | (0x80000000 >> 21); /* Set also FFRA */
	cpu_info.WBBR_LOW = val;
	cpu_info.PC = 0xF7FB8000;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

/* Access (read) a memory location */

#if 0
static int powerpc_once_mem_read(struct target *target, uint32_t addr, uint32_t size, uint32_t *val)
{
	int err;
	struct cpu_status cpu_info;
	uint32_t gpr_scratch_value;
	uint32_t is_vle;
	uint32_t opcode;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	/* Read scratch register and save it */
	err = powerpc_once_gpr_read(target, GPR_SCRATCH, &gpr_scratch_value);
	if (err != ERROR_OK)
		return err;

	memset(&cpu_info, 0, sizeof(cpu_info));

	is_vle = IS_VLE(powerpc->cpu_info.CTL);

	/* Issue a single step load instruction */
	cpu_info.WBBR_LOW = addr;
	cpu_info.CTL = (0x80000000 >> 30) | (0x80000000 >> 21); /* Set also FFRA */

	switch(size) {
	case 1:
		if (is_vle)
			opcode = BUILD_INSTR(POWERPC_E_LBZ_OPCODE, GPR_SCRATCH, 0, 0);
		else
			opcode = BUILD_INSTR(POWERPC_LBZ_OPCODE, GPR_SCRATCH, 0, 0);
		break;
	case 2:
		if (is_vle)
			opcode = BUILD_INSTR(POWERPC_E_LHZ_OPCODE, GPR_SCRATCH, 0, 0);
		else
			opcode = BUILD_INSTR(POWERPC_LHZ_OPCODE, GPR_SCRATCH, 0, 0);
		break;
	case 4:
		if (is_vle)
			opcode = BUILD_INSTR(POWERPC_E_LWZ_OPCODE, GPR_SCRATCH, 0, 0);
		else
			opcode = BUILD_INSTR(POWERPC_LWZ_OPCODE, GPR_SCRATCH, 0, 0);
		break;
	default:
		return -1;
	}

	/* Set the instruction register */
	cpu_info.IR = opcode;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Read CPU info */
	memset(&cpu_info, 0, sizeof(cpu_info));
	err = powerpc_once_read_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	*val = cpu_info.WBBR_LOW;

	/* Restore scratch register value */
	err = powerpc_once_gpr_write(target, GPR_SCRATCH, gpr_scratch_value);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}


/* Access (write) a memory location */
static int powerpc_once_mem_write(struct target *target, uint32_t addr, uint32_t size, uint32_t val, uint32_t is_vle, struct cpu_status cpu_info)
{
	int err;
	uint32_t opcode;

	/* Write value to scratch register */
	err = powerpc_once_gpr_write(target, GPR_SCRATCH, val);
	if (err != ERROR_OK)
		return err;

	/* issue a single step store instruction */
	cpu_info.WBBR_LOW = addr;
	cpu_info.CTL = (0x80000000 >> 30) | (0x80000000 >> 21); /* Set also FFRA */

	switch(size) {
	case 1:
		if (is_vle)
			opcode = BUILD_INSTR(POWERPC_E_STB_OPCODE, GPR_SCRATCH, 0, 0);
		else
			opcode = BUILD_INSTR(POWERPC_STB_OPCODE, GPR_SCRATCH, 0, 0);
		break;
	case 2:
		if (is_vle)
			opcode = BUILD_INSTR(POWERPC_E_STH_OPCODE, GPR_SCRATCH, 0, 0);
		else
			opcode = BUILD_INSTR(POWERPC_STH_OPCODE, GPR_SCRATCH, 0, 0);
		break;
	case 4:
		if (is_vle)
			opcode = BUILD_INSTR(POWERPC_E_STW_OPCODE, GPR_SCRATCH, 0, 0);
		else
			opcode = BUILD_INSTR(POWERPC_STW_OPCODE, GPR_SCRATCH, 0, 0);
		break;
	default:
		return -1;
	}

	/* set the instruction register */
	cpu_info.IR = opcode;

	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

#endif

static int powerpc_once_target_status(struct jtag_tap *tap)
{
	int err;
	uint32_t status_reg;

	/* Read the status register */
	err = powerpc_once_read_status(tap, &status_reg);
	if (err != ERROR_OK)
		return TARGET_UNKNOWN;

	/* Check expected bits are always 1's */
	if ((status_reg & ONCE_IS_DEFAULT) != ONCE_IS_DEFAULT)
		return TARGET_UNKNOWN;
	
	/* Verify correct status */
	if (status_reg & ONCE_IS_RESET)
		return TARGET_RESET;

	if (status_reg & ONCE_IS_ERROR)
		return TARGET_UNKNOWN;

	if (status_reg & ONCE_IS_HALT)
		return TARGET_HALTED;
	
	if (status_reg & ONCE_IS_DEBUG)
		return TARGET_HALTED;

	return TARGET_RUNNING;
}

/*
 * Routine to enter in debug mode during the RESET pin is asserted.
 *
 * This can be called at start time or after a reset request then to
 * be sure to start always in a consistent status register a 
 * set of jtag reset sleeps are necessary.
 */
static int powerpc_once_debug_enter(struct target *target)
{
	int err;
	uint32_t value;

	err = powerpc_once_release(target);
	if (err != ERROR_OK)
		return err;

	/* Set OCR[DR] and OCR[WKUP] */
	value = 0x00000005;
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_OCR, &value);
	if (err != ERROR_OK)
		return err;

	/*Verify exit from reset*/
	for (int i=0; i<100; i++) {
		/* We don't know how many time is the reset pin is asserted ...
		 * How much time human pressed reset button :-) wait a bit.
		 */
		if (powerpc_once_target_status(target->tap) == TARGET_RESET) {
			adapter_deassert_reset();
			jtag_add_sleep(100000);
			jtag_execute_queue();
			/* Enable warning after a long wait time */
			if (i>50) LOG_WARNING("Waiting OnCE to leave reset mode");
		}
		else break;
	}

	/* Latest check before to return error */
	if (powerpc_once_target_status(target->tap) == TARGET_RESET) {
		LOG_INFO("Target no left the reset mode.");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Enable External Debug Mode */

	/*Set DBCR0[EDM] */
	value = 0x80000000;
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_EDBCR0, &value);
	if (err != ERROR_OK)
		return err;

	/*Clear DBSR */
	value = 0xFFFFFFFF;
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_EDBSR0, &value);
	if (err != ERROR_OK)
		return err;


	/* Save CPU context */
	err = powerpc_save_context(target);
	if (err != ERROR_OK)
		return err;

	/* Get and print the TAP IDCODE discovered. */
	powerpc_once_get_idcode(target);

	return ERROR_OK;
}

static int powerpc_write_debug_info_bits(struct target *target,
	uint32_t set_on, uint32_t set_off)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);

	/* Set off bits */
	powerpc->ctx_info &= ~(set_off);

	/* Set on bits */
	powerpc->ctx_info |= set_on;

	return ERROR_OK;
}

static int powerpc_once_debug_exit(struct target *target)
{
	int err;
	uint32_t value;
	struct powerpc_common *powerpc = target_to_powerpc(target);
	struct reg *pc = powerpc->pc;

	uint32_t status_reg;

	/* Read the status register */
	err = powerpc_once_read_status(target->tap, &status_reg);

	LOG_DEBUG("Target status: 0x%08" PRIx32, status_reg);

	/*Clear DBSR */
	value = 0xFFFFFFFF;
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_EDBSR0, &value);
	if (err != ERROR_OK)
		return err;

	/* Clear OCR[DMDIS] and OCR[DR], leave OCR[MCLK] and OCR[FBD] set */
	value = 0x00000006;
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_OCR, &value);
	if (err != ERROR_OK)
		return err;

	/* Adjust PC in case of first execution */
	uint32_t pc_value = buf_get_u32(pc->value, 0, 32);
	if (powerpc->ctx_info & DEBUG_SOFT_BREAK) {
		/* Starting from software debug break was hit */
		powerpc->cpu_info.PC = pc_value - 4;
		powerpc->cpu_info.IR = BUILD_INSTR(POWERPC_E_ORI_OPCODE, 0, 0, 0xD000);
		powerpc->cpu_info.CTL = (0x80000000 >> 30);
		powerpc_write_debug_info_bits(target, DEBUG_RUNNING, 0);
	} else if (powerpc->ctx_info & DEBUG_RUNNING) {
		powerpc->cpu_info.PC = pc_value;
		powerpc_once_reset_cpu_scan_regs(target);
	} else {
		powerpc->cpu_info.PC = pc_value - 4;
		powerpc->cpu_info.IR = BUILD_INSTR(POWERPC_E_ORI_OPCODE, 0, 0, 0xD000);
		powerpc->cpu_info.CTL = (0x80000000 >> 30);
		powerpc_write_debug_info_bits(target, DEBUG_RUNNING, 0);
	}

	/* Set CPUSCR to resume status */
	err = powerpc_once_write_cpu_scan_regs(target->tap, &powerpc->cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Go! */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 1 /*ex*/);
	if (err != ERROR_OK)
		return err;

//	err = powerpc_once_read_status(target->tap, &status_reg);

//	LOG_INFO("Target status: 0x%08" PRIx32, status_reg);

	return ERROR_OK;
}

/**************************************************************************/
/* End OnCE implementation                                                */
/**************************************************************************/

static int powerpc_handle_debug_event(struct target *target)
{
	uint32_t value;
	uint32_t dbg_status_reg;

	/* Save CPU context */
	int err = powerpc_save_context(target);
	if (err != ERROR_OK)
		return err;

	dbg_status_reg = 0;
	err = powerpc_once_reg_read(target->tap, 1, E200Zxx_ONCE_DBSR, &dbg_status_reg);
	if (err != ERROR_OK)
		return err;

	/* Clear status register */
	value = 0xFFFFFFFF;
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_DBSR, &value);
	if (err != ERROR_OK)
		return err;

	/* Handle debug events */
	if (dbg_status_reg & 0x00FF0000) {
		/* hardware breakpoint or watchpoint */
		/* Nothing to do ... */
	} else {
		/* Probably software breakpoint */
		struct powerpc_common *powerpc = target_to_powerpc(target);
		struct reg *pc_reg = powerpc->pc;
		struct breakpoint *breakpoint;
		uint32_t pc;

		/*
		 * Software breakpoint, that is an internal debug event,
		 * we do not know where we are, need to read PC
		 */
		pc = powerpc->core_regs[PC];
		pc -= 2;

		breakpoint = breakpoint_find(target, pc);
		if (breakpoint && (breakpoint->type == BKPT_SOFT)) {
			buf_set_u32(pc_reg->value, 0, 32, pc);
			pc_reg->dirty = true;
			pc_reg->valid = true;
			powerpc_write_debug_info_bits(target, DEBUG_SOFT_BREAK, 0);
			LOG_INFO("Signaling Software breakpoint");
			target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);
		}
	}

	return ERROR_OK;
}


static const struct jim_nvp powerpc_target_state[] = {
	{ .name = "unknown", .value = TARGET_UNKNOWN },
	{ .name = "running", .value = TARGET_RUNNING },
	{ .name = "halted",  .value = TARGET_HALTED },
	{ .name = "reset",   .value = TARGET_RESET },
	{ .name = "debug-running", .value = TARGET_DEBUG_RUNNING },
	{ .name = NULL, .value = -1 },
};


static int powerpc_poll(struct target *target)
{
	int err;
	enum target_state previous_target_state = target->state;
	enum target_state current_target_state;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	current_target_state = powerpc_once_target_status(target->tap);

	if (current_target_state != previous_target_state) {
		const char *previous_state;
		const char *current_state;

		previous_state = jim_nvp_value2name_simple(powerpc_target_state, previous_target_state)->name;
		if (!previous_state)
			previous_state = "undefined (BUG ?)";

		current_state = jim_nvp_value2name_simple(powerpc_target_state, current_target_state)->name;
		if (!current_state)
			current_state = "undefined (BUG ?)";

		LOG_DEBUG("(%s) target state changed: %s --> %s", __func__, previous_state, current_state);
	}


	if (current_target_state == TARGET_UNKNOWN) {
		target->state = TARGET_UNKNOWN;
		LOG_ERROR("jtag status contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	/*
	 * If current state is HALTED and previous state is UNKNOWN, we have been
	 * connected with the target already in DEBUG mode. Force Entering... in debug mode.
	 *
	 * If target is running and previous state is UNKNOWN, we have been
	 * connected with the target in running mode. Force Entering... in debug mode.
	 *
	 */
	if (((current_target_state  == TARGET_HALTED)  && (previous_target_state == TARGET_UNKNOWN)) ||
			(((current_target_state == TARGET_RUNNING) && (previous_target_state == TARGET_UNKNOWN)))) {

		/* Do we have a request to halt ? (see powerpc_target_create) */
		if (powerpc->ctx_info & DEBUG_HALT) {

			/* We decided to stay always in debug mode */
			err = powerpc_once_debug_enter(target);
			if (err != ERROR_OK) {
				LOG_INFO("ERROR");
				return err;
			}

			/* Set the HALTED state */
			target->state = TARGET_HALTED;

			powerpc_write_debug_info_bits(target, 0, DEBUG_HALT); // set debug_halt off?

			if (previous_target_state == TARGET_DEBUG_RUNNING)
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			else
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);

			target->debug_reason = DBG_REASON_DBGRQ;

			LOG_INFO("Target status: halted - PC: 0x%08" PRIx32, powerpc->cpu_info.PC);
		}
	} else if (current_target_state == TARGET_RUNNING) {

		/* Do we have a request to halt ? */
		if (powerpc->ctx_info & DEBUG_HALT) {
			/* Enter in debug mode, target state will be HALTED */

			/* Be aware: this function can modify the target->state */
			err = powerpc_once_debug_enter(target);
			if (err != ERROR_OK)
				return err;
			target->state = TARGET_HALTED;

			/* Clear the halt request */
			powerpc_write_debug_info_bits(target, 0, DEBUG_HALT);

			if (target->state == TARGET_HALTED) {


				if (previous_target_state == TARGET_DEBUG_RUNNING)
					target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
				else
					target_call_event_callbacks(target, TARGET_EVENT_HALTED);

				target->debug_reason = DBG_REASON_DBGRQ;

				LOG_INFO("Target status: halted - PC: 0x%08" PRIx32, powerpc->cpu_info.PC);
			}
		} else {
			if (previous_target_state == TARGET_DEBUG_RUNNING)
				target->state = TARGET_DEBUG_RUNNING;
			else
				/* Set target state to running */
				target->state = TARGET_RUNNING;
		}

	} else if (current_target_state == TARGET_HALTED) {

		/* Set target state to halted */
		target->state = TARGET_HALTED;

		if (powerpc->ctx_info & DEBUG_STEP) {
			/* Clear the step request */
			powerpc_write_debug_info_bits(target, 0, DEBUG_STEP);
		} else if (previous_target_state == TARGET_DEBUG_RUNNING) {
			powerpc_write_debug_info_bits(target, DEBUG_HALT, DEBUG_RUNNING);
		} else {
			if (previous_target_state == TARGET_RUNNING) {
				/* Breakpoint or watchpoint hit */
				err = powerpc_handle_debug_event(target);
				if (err != ERROR_OK)
					return err;

				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
		}
	} else if (current_target_state == TARGET_RESET) {
		target->state = TARGET_RESET;
	}

	/* 
	 * We MUST release the OnCE TAP controller in order the main JTAGC
	 *	can be correct probed during a possible jtag scan chain reset request.
	 */
	err = powerpc_once_release(target);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int powerpc_halt(struct target *target)
{
	enum target_state previous_target_state = target->state;

	LOG_DEBUG("(%s) Entering: %s",  __func__, target_state_name(target));
	LOG_DEBUG("target->state: %s",  target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	powerpc_once_debug_enter(target);

	/* Write to Debug Halting Control and Status Register */
	powerpc_write_debug_info_bits(target, DEBUG_HALT, DEBUG_RUNNING);

	target->state = TARGET_HALTED;

	if ((previous_target_state == TARGET_DEBUG_RUNNING) || (previous_target_state == TARGET_RESET))
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
	else
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	target->debug_reason = DBG_REASON_DBGRQ;

	LOG_DEBUG("(%s) Exiting: %s",  __func__, target_state_name(target));

	return ERROR_OK;
}

static int powerpc_soft_reset_halt(struct target *target)
{
	LOG_WARNING("soft_reset_halt is deprecated, please use 'reset halt' instead.");

	return ERROR_OK;
}

#if 0
static void powerpc_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* Set any pending breakpoints */
	while (breakpoint) {
		if (!breakpoint->is_set)
			powerpc_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

static void powerpc_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	/* Set any pending watchpoints */
	while (watchpoint) {
		if (!watchpoint->is_set)
			powerpc_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}
#endif

static int powerpc_resume(struct target *target, int current,
	target_addr_t address, int handle_breakpoints, int debug_execution)
{
	int err;
	struct powerpc_common *powerpc = target_to_powerpc(target);
	struct reg *pc = powerpc->pc;
	struct breakpoint *breakpoint = NULL;

	LOG_DEBUG("(%s) Entering: %s",  __func__, target_state_name(target));

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u32(pc->value, 0, 32, address);
		pc->dirty = true;
		pc->valid = true;
	}

	uint32_t pc_value = buf_get_u32(pc->value, 0, 32);

	/* The front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, pc_value);
		if (breakpoint)
			powerpc_unset_breakpoint(target, breakpoint);
	}

	/* Restore CPU Context */
	err = powerpc_restore_context(target);
	if (err != ERROR_OK)
		return err;

	/* Exit debug mode */
	err = powerpc_once_debug_exit(target);
	if (err != ERROR_OK)
		return err;

	target->debug_reason = DBG_REASON_NOTHALTED;

	/* Registers are now invalid */
	register_cache_invalidate(powerpc->core_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32 "", powerpc->cpu_info.PC);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32 "", powerpc->cpu_info.PC);
	}

	powerpc_write_debug_info_bits(target, DEBUG_RUNNING, DEBUG_HALT);

	LOG_DEBUG("(%s) Exiting: %s",  __func__, target_state_name(target));

	return ERROR_OK;
}

static int powerpc_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int err;
	uint32_t is_vle;
	uint16_t instruction;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	LOG_DEBUG("BPID: %" PRIu32 ", Type: %d [0=hb, 1=b], Address: 0x%08" PRIx64 " Length: %d (set=%d)",
			breakpoint->unique_id,
			(int)(breakpoint->type),
			breakpoint->address,
			breakpoint->length,
			breakpoint->is_set);

	if (!breakpoint->is_set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		uint32_t iac_reg;
		uint32_t value;

		/* Read the debug control register value */
		err = powerpc_once_reg_read(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
		if (err != ERROR_OK)
			return err;

		/* Retrieve the IAC index used to set this hw_breakpoint */
		iac_reg = breakpoint->number - 1;

		/* Disable comparator */
		if (iac_reg < 4) {
			value &= ~(0x00800000 >> iac_reg);
		} else {
			value &= ~(0x4000 >> (iac_reg -4));
		}

		err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
		if (err != ERROR_OK)
			return err;

		err = powerpc_once_reg_read(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
		if (err != ERROR_OK)
			return err;

		LOG_DEBUG("E200Zxx_ONCE_DBCR0 0x%08" PRIx32, value);

		/* Get IAC available again */
		powerpc->hwb.free++;
		powerpc->hwb.mask |= (1 << iac_reg);
	} else {
		/* Software breakpoint */
		is_vle = IS_VLE(powerpc->cpu_info.CTL);
		if (is_vle)
		{
			if (((breakpoint->length == 4) && (breakpoint->address & 0x3u)))
			{
				 if (!(breakpoint->address & 0x1u)){

					 instruction = (breakpoint->orig_instr[0] << 8) | breakpoint->orig_instr[1];
					 if((instruction & 0x9000) == 0x1000)
					 {
						 // first 4 bits have a value of 1,3,5,7
						 //instruction was 32-bit
						 err = target_write_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr);
						 if (err != ERROR_OK)
							 return err;

						 err = target_write_memory(target, breakpoint->address + 2, 2, 1, breakpoint->orig_instr + 2);
						 if (err != ERROR_OK)
							 return err;
					 }
					 else
					 {
						 err = target_write_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr);
						 if (err != ERROR_OK)
						    return err;
					 }


				 }
			}
			else
			{
				err = target_write_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr);
				if (err != ERROR_OK)
						return err;
			}

		/* Restore original instruction (kept in target endianness) */
		}
		else {
			err = target_write_memory(target, breakpoint->address, breakpoint->length, 1,
				breakpoint->orig_instr);
			if (err != ERROR_OK)
				return err;
		}
	}

	breakpoint->is_set = false;

	return ERROR_OK;
}

static int powerpc_set_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int err;
	int32_t iac_reg;
	uint32_t is_vle;
	uint16_t instruction;
	int i;
	uint32_t value;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	LOG_DEBUG("BPID: %" PRIu32 ", Type: %d [0=hb, 1=b], Address: 0x%08" PRIx64 " Length: %d (set=%d)",
				breakpoint->unique_id,
				(int)(breakpoint->type),
				breakpoint->address,
				breakpoint->length,
				breakpoint->is_set);

	if (breakpoint->is_set) {
		LOG_WARNING("breakpoint (BPID: %" PRIu32 ") already set", breakpoint->unique_id);
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {

		if (powerpc->hwb.free < 1) {
			LOG_WARNING("no more hardware breakpoints available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		/* Get the first available IAC register */
		iac_reg = -1;
		for (i = 0; i < powerpc->hwb.avail; i++) {
			if (powerpc->hwb.mask & (1 << i)) {
				iac_reg = i;
				break;
			}
		}

		if (iac_reg < 0) {
			LOG_WARNING("no more hardware breakpoints available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		value = breakpoint->address;
		if (iac_reg < 4) {
			err = powerpc_once_reg_write(target->tap, 1, (iac_reg + E200Zxx_ONCE_IAC1), &value);
			if (err != ERROR_OK)
				return err;
		} else {
			err = powerpc_once_reg_write(target->tap, 1, (iac_reg - 4 + E200Zxx_ONCE_IAC5), &value);
			if (err != ERROR_OK)
				return err;
		}


		err = powerpc_once_reg_read(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
		if (err != ERROR_OK)
			return err;

		/* Enable comparator */
		if (iac_reg < 4) {
			value |= (0x00800000 >> iac_reg);
		} else {
			value |= (0x4000 >> (iac_reg-4));
		}
		err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
		if (err != ERROR_OK)
			return err;

		/*
		 * Use the set field to store the debug comparator
		 * register index. It will be used when removing the hw_breakpoint.
		 */
		breakpoint_hw_set(breakpoint, iac_reg + 1);

		/* IAC used, one less */
		powerpc->hwb.free--;
		powerpc->hwb.mask &= ~(1 << iac_reg);

	} else if (breakpoint->type == BKPT_SOFT) {

		uint8_t code[4];

		buf_set_u32(code, 0, 32, POWERPC_BKPT_INST);

		is_vle = IS_VLE(powerpc->cpu_info.CTL);

		err = target_read_memory(target,
				breakpoint->address,
				2, 1,
				breakpoint->orig_instr);
		if (err != ERROR_OK)
			return err;

		if(is_vle)
		{
		 instruction = (breakpoint->orig_instr[0] << 8) | breakpoint->orig_instr[1];
		 if((instruction & 0x9000) == 0x1000)
		 {
			 // first 4 bits have a value of 1,3,5,7
			 //instruction was 32-bit
			 err = target_read_memory(target,
			 			 						breakpoint->address + 2,
			 			 						2, 1,
			 			 						breakpoint->orig_instr + 2);
			 if (err != ERROR_OK)
			 			return err;

			 if (((breakpoint->length == 4) && (breakpoint->address & 0x3u))){
			 		 if (!(breakpoint->address & 0x1u)){
			 			err = target_write_memory(target,
			 										breakpoint->address,
			 						 				2, 1,
			 						 				code);
			 			if (err != ERROR_OK)
			 				return err;

			 			err = target_write_memory(target,
			 						 			  breakpoint->address + 2,
			 						 			  2, 1,
			 						 			  code);
			 			if (err != ERROR_OK)
			 				return err;
			 		 }
			 }else {
					 err = target_write_memory(target,
					 				breakpoint->address,
					 				breakpoint->length, 1,
					 				code);
			 		 if (err != ERROR_OK)
						 			return err;
			 }



		 }
		 else
		 {
			 // first 4 bits have a value of 0,2,4,6,8,9,A,B,C,D,E (and F, but F is reserved)
			 //instruction was 16-bit
			 breakpoint->length = 2;
			 err = target_write_memory(target,
			 				breakpoint->address,
			 				breakpoint->length, 1,
			 				code);
			 		if (err != ERROR_OK)
			 			return err;
		 }
		}

		breakpoint->is_set = true;
	}

	return ERROR_OK;
}

static int powerpc_step(struct target *target, int current,
	target_addr_t address, int handle_breakpoints)
{
	int err;
	struct cpu_status cpu_info;
	struct powerpc_common *powerpc = target_to_powerpc(target);
	struct reg *pc = powerpc->pc;
	struct breakpoint *breakpoint = NULL;
	uint32_t saved_MSR;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(pc->value, 0, 32, address);

	uint32_t pc_value = buf_get_u32(pc->value, 0, 32);

	/* The front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, pc_value);
		if (breakpoint)
			powerpc_unset_breakpoint(target, breakpoint);
	}

	memcpy(&cpu_info, &powerpc->cpu_info, sizeof(cpu_info));

	saved_MSR = cpu_info.MSR;

	/* Verify if there is a software breakpoint */
	if (!cpu_info.IR) {
		uint32_t buf;
		powerpc->ctx_info |= DEBUG_SOFT_BREAK;
		target_read_memory(target, pc_value, 4, 1, (uint8_t *)&buf);
		cpu_info.IR = buf;
	}

	if (powerpc->ctx_info & DEBUG_SOFT_BREAK) {
		/* Steping from software debug break was hit */
		cpu_info.PC = pc_value - 4;
		cpu_info.IR = BUILD_INSTR(POWERPC_E_ORI_OPCODE, 0, 0, 0xD000);
		cpu_info.CTL = (0x80000000 >> 30);
		powerpc_write_debug_info_bits(target, 0, DEBUG_SOFT_BREAK);
		powerpc_write_debug_info_bits(target, DEBUG_STEP_TWICE, 0);
	} else if (powerpc->ctx_info & DEBUG_RUNNING) {
		/* Normal running */
		cpu_info.PC = pc_value;
		powerpc_once_reset_cpu_scan_regs(target);
	} else {
		/* Adjust program counter if first execution */
		cpu_info.PC = pc_value - 4;
		cpu_info.IR = BUILD_INSTR(POWERPC_E_ORI_OPCODE, 0, 0, 0xD000);
		cpu_info.CTL = (0x80000000 >> 30);
		powerpc_write_debug_info_bits(target, DEBUG_RUNNING, 0);
		powerpc_write_debug_info_bits(target, DEBUG_STEP_TWICE, 0);
	}

	target->debug_reason = DBG_REASON_SINGLESTEP;

	err = powerpc_restore_context(target);
	if (err != ERROR_OK)
		return err;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

do_again:
	/* Clear MSR disable breakpoint during step by step*/
	cpu_info.MSR = 0;

	/* Write CPU info */
	err = powerpc_once_write_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Step */
	err = powerpc_once_exec(target->tap, 1 /*go*/, 0 /*ex*/);
	if (err != ERROR_OK)
		return err;

	/* Read CPU info */
	err = powerpc_once_read_cpu_scan_regs(target->tap, &cpu_info);
	if (err != ERROR_OK)
		return err;

	if (powerpc->ctx_info & DEBUG_STEP_TWICE) {
		powerpc_write_debug_info_bits(target, 0, DEBUG_STEP_TWICE);

		/* Jumping back */
		goto do_again;
	}

	memcpy(&powerpc->cpu_info, &cpu_info, sizeof(struct cpu_status));

	/* Registers are now invalid */
	register_cache_invalidate(powerpc->core_cache);

	if (breakpoint)
		powerpc_set_breakpoint(target, breakpoint);

	err = powerpc_save_context(target);
	if (err != ERROR_OK)
		return err;

	/* restore MSR */
	powerpc->cpu_info.MSR = saved_MSR;

	/* Write to Debug Halting Control and Status Register */
	powerpc_write_debug_info_bits(target, DEBUG_STEP, 0);

	target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);

	return ERROR_OK;
}

/* 
 * Routine called after the JTAGC reset was done into
 * JTAG common reset interface. This reset all TAPs like CPU.
 */ 
static int powerpc_assert_reset(struct target *target)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);

	LOG_DEBUG("(%s) Entering: %s",  __func__, target_state_name(target));
	LOG_DEBUG("target->state: %s",  target_state_name(target));

	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		/* allow scripts to override the reset event */
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
		register_cache_invalidate(powerpc->core_cache);
		target->state = TARGET_RESET;

		return ERROR_OK;
	}

	target->state = TARGET_RESET;
	register_cache_invalidate(powerpc->core_cache);

	//powerpc_jtag_send_ir(target->tap, JTAG_INSTR_EXTEST);
	if (target->reset_halt) {
		adapter_assert_reset();
		powerpc_halt(target);
	}

	return ERROR_OK;
}

static int powerpc_deassert_reset(struct target *target)
{
	LOG_DEBUG("(%s) Entering: %s",  __func__, target_state_name(target));
	LOG_DEBUG("target->state: %s",  target_state_name(target));

	/* deassert reset lines */
//	adapter_deassert_reset();

	return ERROR_OK;
}

static int powerpc_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	return powerpc_set_breakpoint(target, breakpoint);
}

static int powerpc_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->is_set)
		powerpc_unset_breakpoint(target, breakpoint);

	return ERROR_OK;
}

static int powerpc_set_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	int err;
	int i;
	int32_t dac_reg;
	uint32_t value;
	uint32_t function;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	if (powerpc->dwp.free < 1) {
		LOG_WARNING("no more hardware watchpoints available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Get the first available DAC register */
	dac_reg = -1;
	for (i = 0; i < powerpc->dwp.avail; i++) {
		if (powerpc->dwp.mask & (1 << i)) {
			dac_reg = i;
			break;
		}
	}

	if (dac_reg < 0) {
		LOG_WARNING("no more hardware watchpoints available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	value = watchpoint->address;
	err = powerpc_once_reg_write(target->tap, 1, (dac_reg + E200Zxx_ONCE_DAC1), &value);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_reg_read(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
	if (err != ERROR_OK)
		return err;

	switch (watchpoint->rw) {
	case WPT_READ:
		function = 0x00080000;
		break;
	case WPT_WRITE:
		function = 0x00040000;
		break;
	case WPT_ACCESS:
		function = 0x000C0000;
		break;
	default:
		function = 0; /* Disabled */
		break;
	}

	/* Enable comparator */
	value &= ~(0x000C0000 >> (dac_reg * 2));
	value |=  (function >> (dac_reg * 2));
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
	if (err != ERROR_OK)
		return err;

	/*
	 * Use the set field to store the debug comparator
	 * register index. It will be used when removing the watchpoint.
	 */
	watchpoint_set(watchpoint, dac_reg + 1);

	/* DAC used, one less */
	powerpc->dwp.free--;
	powerpc->dwp.mask &= ~(1 << dac_reg);

	return ERROR_OK;
}

static int powerpc_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("Watchpoint (ID %d), addr = 0x%08" PRIx64 ", len = %d, mask = 0x%08x, value = 0x%08x, RW: %d",
		watchpoint->unique_id,
		watchpoint->address,
		watchpoint->length,
		watchpoint->mask,
		watchpoint->value,
		watchpoint->rw);

	return powerpc_set_watchpoint(target, watchpoint);
}

static int powerpc_unset_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	int err;
	uint32_t dac_reg;
	uint32_t value;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	if (!watchpoint->is_set) {
		LOG_WARNING("watchpoint (wpid: %d) not set", watchpoint->unique_id);
		return ERROR_OK;
	}

	/* Read the debug control register value */
	err = powerpc_once_reg_read(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
	if (err != ERROR_OK)
		return err;

	/* Retrieve the DAC index used to set this watchpoint */
	dac_reg = watchpoint->number - 1;

	/* Disable comparator */
	value &= ~(0x000C0000 >> (dac_reg * 2));
	err = powerpc_once_reg_write(target->tap, 1, E200Zxx_ONCE_DBCR0, &value);
	if (err != ERROR_OK)
		return err;

	/* Watchpoint no more set */
	watchpoint->is_set = false;

	/* Get DAC available */
	powerpc->dwp.free++;
	powerpc->dwp.mask |= (1 << dac_reg);

	return ERROR_OK;
}

static int powerpc_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return powerpc_unset_watchpoint(target, watchpoint);
}

static int powerpc_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int err;

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
			address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 8) && (address & 0x7u)) || ((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Read memory ... */
	err = powerpc_nexus_mem_read(target, address, size, count, (uint32_t *)(void *)buffer);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int powerpc_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int err;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 8) && (address & 0x7u)) ||((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Write memory ... */
	err = powerpc_nexus_mem_write(target, address, size, count, (uint32_t *)(void *)buffer);
	if (err != ERROR_OK)
		return err;
	return ERROR_OK;
}

#if 0
static int powerpc_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int err;

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
			address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	uint32_t i;
	uint8_t *d = buffer;
	uint32_t val;

	/* Read memory ... */
	for (i = 0; i < count; i++) {
		uint8_t *s = (uint8_t *) &val;

		s += (size - 1);

		err = powerpc_once_mem_read(target, address, size, &val);
		if (err != ERROR_OK)
			return err;

		/*
		 * TODO:
		 *    Avoid data copy.
		 *    Check if address of input buffer is aligned to
		 *    requested size and use it in OnCE read call.
		 */
		*d++ = *s--;
		if (size > 1)
			*d++ = *s--;
		if (size > 2) {
			*d++ = *s--;
			*d++ = *s--;
		}
		address += size;
	}
	return ERROR_OK;
}



static int powerpc_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int err;
	uint32_t is_vle;
	uint32_t gpr_scratch_value;
	struct cpu_status cpu_info;

	struct powerpc_common *powerpc = target_to_powerpc(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Write memory ... */

	uint32_t i;
	const uint8_t *s = buffer;
	uint32_t val;
/*
	if ((count * size) < 8)
	{
		err = powerpc_nexus_mem_write(target, address, size, count, (uint8_t *) &buffer);
		if (err != ERROR_OK)
			return err;
	}

	return ERROR_OK;
*/
	is_vle = IS_VLE(powerpc->cpu_info.CTL);

	/* Read scratch register and save it */
	err = powerpc_once_gpr_read(target, GPR_SCRATCH, &gpr_scratch_value);
	if (err != ERROR_OK)
		return err;

	memset(&cpu_info, 0, sizeof(cpu_info));

	for (i = 0; i < count; i++) {
		uint8_t *d = (uint8_t *) &val;

		d += (size - 1);

		/*
		 * TODO:
		 *    Avoid data copy.
		 *    Check if address of input buffer is aligned to
		 *    requested size and use it in OnCE write call.
		 */
		*d-- = *s++;
		if (size > 1)
			*d-- = *s++;
		if (size > 2) {
			*d-- = *s++;
			*d-- = *s++;
		}

		err = powerpc_once_mem_write(target, address, size, val, is_vle, cpu_info);
		if (err != ERROR_OK)
			return err;

		address += size;
	}

	/* Restore scratch register value */
	err = powerpc_once_gpr_write(target, GPR_SCRATCH, gpr_scratch_value);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

#endif


static int powerpc_save_context(struct target *target)
{
	int err;
	unsigned int i;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	/*
	 * Read CPU info
	 * Ref. AN4035 3.9
	 * Catch the CPUSCR registers is mandatory for a correct
	 * Restore Context when exit from debug mode when Entering...
	 * in debug mode.
	 */

	err = powerpc_once_read_cpu_scan_regs(target->tap, &powerpc->cpu_info);
	if (err != ERROR_OK)
		return err;

	/* Read core registers */
	for (i = 0; i < POWERPC_NUM_GP_REGS; i++) {
		err = powerpc_once_gpr_read(target, i, &powerpc->core_regs[i]);
		if (err != ERROR_OK)
			return err;
	}

	/* PC and MSR register already available in the cpu_info structure */
	powerpc->core_regs[PC] = powerpc->cpu_info.PC;
	powerpc->core_regs[MSR] = powerpc->cpu_info.MSR;

	err = powerpc_once_cr_read(target, &powerpc->core_regs[CR]);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_spr_read(target, SPR_LR, &powerpc->core_regs[LR]);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_spr_read(target, SPR_CTR, &powerpc->core_regs[CTR]);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_spr_read(target, SPR_XER, &powerpc->core_regs[XER]);
	if (err != ERROR_OK)
		return err;

	for (i = 0; i < POWERPC_NUM_REGS; i++) {
		if (!powerpc->core_cache->reg_list[i].valid)
			powerpc->read_core_reg(target, i);
	}

	return ERROR_OK;
}

static int powerpc_restore_context(struct target *target)
{
	int err;
	unsigned int i;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	/* Restore the GP core registers */
	for (i = 0; i < POWERPC_NUM_GP_REGS; i++) {

		if (powerpc->core_cache->reg_list[i].dirty)
			powerpc->write_core_reg(target, i);

		err = powerpc_once_gpr_write(target, i, powerpc->core_regs[i]);
		if (err != ERROR_OK)
			return err;
	}

	/* Restore XER SPR register */
	if (powerpc->core_cache->reg_list[XER].dirty)
		powerpc->write_core_reg(target, XER);

	err = powerpc_once_spr_write(target, SPR_XER, powerpc->core_regs[XER]);
	if (err != ERROR_OK)
		return err;

	/* Restore LR SPR register */
	if (powerpc->core_cache->reg_list[LR].dirty)
		powerpc->write_core_reg(target, LR);

	err = powerpc_once_spr_write(target, SPR_LR, powerpc->core_regs[LR]);
	if (err != ERROR_OK)
		return err;

	/* Restore CTR SPR register */
	if (powerpc->core_cache->reg_list[CTR].dirty)
		powerpc->write_core_reg(target, CTR);

	err = powerpc_once_spr_write(target, SPR_CTR, powerpc->core_regs[CTR]);
	if (err != ERROR_OK)
		return err;

	/* Restore CR register */
	if (powerpc->core_cache->reg_list[CR].dirty)
		powerpc->write_core_reg(target, CR);

	err = powerpc_once_cr_write(target, powerpc->core_regs[CR]);
	if (err != ERROR_OK)
		return err;

	if (powerpc->core_cache->reg_list[PC].dirty)
		powerpc->write_core_reg(target, PC);

	if (powerpc->core_cache->reg_list[MSR].dirty)
			powerpc->write_core_reg(target, MSR);

    return ERROR_OK;
}

static int powerpc_read_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	if (num >= POWERPC_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = powerpc->core_regs[num];
	buf_set_u32(powerpc->core_cache->reg_list[num].value, 0, 32, reg_value);
	powerpc->core_cache->reg_list[num].valid = 1;
	powerpc->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

static int powerpc_write_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;
	struct powerpc_common *powerpc = target_to_powerpc(target);

	if (num >= POWERPC_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = buf_get_u32(powerpc->core_cache->reg_list[num].value, 0, 32);
	powerpc->core_regs[num] = reg_value;
	powerpc->core_cache->reg_list[num].valid = 1;
	powerpc->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

static int powerpc_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
			int *reg_list_size, enum target_register_class reg_class)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);
	unsigned int i;

	/* Include control registers */
	*reg_list_size = POWERPC_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < POWERPC_NUM_REGS; i++)
		(*reg_list)[i] = &powerpc->core_cache->reg_list[i];

	return ERROR_OK;
}


static int powerpc_get_core_reg(struct reg *reg)
{
	struct powerpc_core_reg *powerpc_reg = reg->arch_info;
	struct target *target = powerpc_reg->target;
	struct powerpc_common *powerpc_target = target_to_powerpc(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	return powerpc_target->read_core_reg(target, powerpc_reg->num);
}

static int powerpc_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct powerpc_core_reg *powerpc_reg = reg->arch_info;
	struct target *target = powerpc_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}


/* Starts an algorithm in the target. */
int powerpc_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);
	struct powerpc_algorithm *powerpc_algorithm_info = arch_info;
	int retval = ERROR_OK;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	if (powerpc_algorithm_info->common_magic != POWERPC_COMMON_MAGIC) {
		LOG_ERROR("current target isn't an POWERPC target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache
	 * Not needed if core register cache is always consistent with target process state */
	for (unsigned i = 0; i < powerpc->core_cache->num_regs; i++) {
		powerpc_algorithm_info->context[i] = buf_get_u32(
				powerpc->core_cache->reg_list[i].value,
				0,
				32);
	}

	/* Memory parameters */
	for (int i = 0; i < num_mem_params; i++) {
		/* Write only out params */
		if (mem_params[i].direction == PARAM_OUT) {
			retval = target_write_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* Register parameters */
	for (int i = 0; i < num_reg_params; i++) {
		struct reg *reg =
			register_get_by_name(powerpc->core_cache, reg_params[i].reg_name, 0);

		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
				reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		powerpc_set_core_reg(reg, reg_params[i].value);
	}

	/*
	 * NOTE: powerpc_run_algorithm requires that each algorithm uses
	 * a software breakpoint at the exit point
	 */

	retval = target_resume(target, 0, entry_point, 1, 1);

	return retval;
}

/* Waits for an algorithm in the target. */
int powerpc_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);
	struct powerpc_algorithm *powerpc_algorithm_info = arch_info;
	int retval;
	int err;
	int i, j;
	//uint32_t pc;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	/*
	 * NOTE: powerpc_run_algorithm requires that each algorithm uses
	 * a software breakpoint at the exit point
	 */

	if (powerpc_algorithm_info->common_magic != POWERPC_COMMON_MAGIC) {
		LOG_ERROR("current target isn't an POWERPC target");
		return ERROR_TARGET_INVALID;
	}

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);

	/* If the target fails to halt due to the breakpoint, force a halt */
	if (retval != ERROR_OK || target->state != TARGET_HALTED) {
		LOG_DEBUG("Halting the target...");
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;
		return ERROR_TARGET_TIMEOUT;
	}

/*
	pc = buf_get_u32(powerpc->pc->value, 0, 32);
	if (exit_point && (pc != exit_point)) {
		LOG_INFO("failed algorithm halted at 0x%016x" PRIx32 ", expected 0x%016llx" PRIx32,
			pc,
			exit_point);
		return ERROR_TARGET_TIMEOUT;
	}
*/

	/* Read memory values to mem_params[] */
	for (i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	for (i = 0; i < POWERPC_NUM_GP_REGS; i++) {
		err = powerpc_once_gpr_read(target, i, &powerpc->core_regs[i]);
		if (err != ERROR_OK)
			return err;
	}

	for (i = powerpc->core_cache->num_regs - 1; i >= 0; i--)
	{
		for (j = 0; j < num_reg_params; j++ )
		{
			if (reg_params[j].direction != PARAM_IN) {
				if (strcmp(powerpc->core_cache->reg_list[i].name, reg_params[j].reg_name) == 0)
				{
					powerpc->read_core_reg(target, i);
				}
			}
		}

	}

	/* Copy core register values to reg_params[] */
	for (i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_IN) {
			struct reg *reg = register_get_by_name(powerpc->core_cache,
					reg_params[i].reg_name,
					0);

			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			if (reg->size != reg_params[i].size) {
				LOG_ERROR(
					"BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	/* Restore original context */
	for (i = powerpc->core_cache->num_regs - 1; i >= 0; i--) {
		uint32_t regvalue;
		regvalue = buf_get_u32(powerpc->core_cache->reg_list[i].value, 0, 32);
		if (regvalue != powerpc_algorithm_info->context[i]) {

			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
				powerpc->core_cache->reg_list[i].name,
				powerpc_algorithm_info->context[i]);

			buf_set_u32(powerpc->core_cache->reg_list[i].value,
				0, 32, powerpc_algorithm_info->context[i]);
			powerpc->core_cache->reg_list[i].valid = 1;
			powerpc->core_cache->reg_list[i].dirty = 1;
		}
	}

	return retval;
}



/* Runs an algorithm on target */
int powerpc_run_algorithm(struct target *target, int num_mem_params,
		                  struct mem_param *mem_params, int num_reg_params,
						  struct reg_param *reg_params, target_addr_t entry_point, target_addr_t exit_point,
						  int timeout_ms, void *arch_info)
{
	int err;

	LOG_DEBUG("%s:%d %s()", __FILE__, __LINE__, __func__);

	err = powerpc_start_algorithm(target,
			num_mem_params, mem_params,
			num_reg_params, reg_params,
			entry_point, exit_point,
			arch_info);

	if (err == ERROR_OK)
		err = powerpc_wait_algorithm(target,
				num_mem_params, mem_params,
				num_reg_params, reg_params,
				exit_point, timeout_ms,
				arch_info);

	return err;
}


int powerpc_arch_state_gen(struct target *target)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);
	if (powerpc->common_magic != POWERPC_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-POWERPC target");
		return ERROR_FAIL;
	}

	/* avoid filling log waiting for fileio reply */
	if (target->semihosting && target->semihosting->hit_fileio)
		return ERROR_OK;

	LOG_USER("target halted");

	return ERROR_OK;
}

/* architecture specific status reply */
static int powerpc_arch_state(struct target *target)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);
	int retval;

	struct reg *pc = powerpc->pc;

	uint32_t pc_value = buf_get_u32(pc->value, 0, 32);

	retval = powerpc_arch_state_gen(target);

	if (target->debug_reason == DBG_REASON_WATCHPOINT)
		LOG_USER("Watchpoint triggered at  PC 0x%8.8" PRIx32,  pc_value);

	return retval;
}


static const struct reg_arch_type powerpc_reg_type = {
	.get = powerpc_get_core_reg,
	.set = powerpc_set_core_reg,
};

struct reg_cache *powerpc_build_reg_cache(struct target *target)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);

	int num_regs = POWERPC_NUM_REGS;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct powerpc_core_reg *arch_info = malloc(sizeof(struct powerpc_core_reg) * num_regs);
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "powerpc registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	powerpc->core_cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = powerpc_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].powerpc_common = powerpc;

		reg_list[i].name = powerpc_regs[i].name;
		reg_list[i].size = 32;

		reg_list[i].value = calloc(1, 4);
		reg_list[i].valid = 0;
		reg_list[i].type = &powerpc_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = powerpc_regs[i].type;
		else
			LOG_ERROR("unable to allocate reg type list");

		reg_list[i].dirty = 0;

		reg_list[i].group = powerpc_regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = powerpc_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");
	}

	powerpc->pc = reg_list + PC;

	return cache;
}

static int powerpc_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	/* Build register cache */
	powerpc_build_reg_cache(target);
	return ERROR_OK;
}

/*
 * Routine called when OpenOCD session is ending
 */
static void powerpc_deinit_target(struct target *target)
{
	struct powerpc_common *powerpc = target_to_powerpc(target);

	LOG_DEBUG("target->state: %s",  target_state_name(target));

	powerpc_once_release(target);

	free(powerpc);

	LOG_INFO("SPC5 0x%08x disconnected.", target->tap->idcode);
}

static int powerpc_examine(struct target *target)
{
	uint32_t idcode;

	if (target->tap->hasidcode == false) {
		LOG_ERROR("no IDCODE present on device");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	idcode = target->tap->idcode;
	if (!target_was_examined(target)) {
		target_set_examined(target);

		if (powerpc_jtag_get_idcode(target) != idcode)
			return ERROR_FAIL;
	}

	powerpc_decode_idcode(target->tap->idcode);

	return ERROR_OK;
}

static int powerpc_target_request_data(struct target *target,
	uint32_t size, uint8_t *buffer)
{
	return ERROR_OK;
}


static int powerpc_target_create(struct target *target, Jim_Interp *interp)
{
	struct powerpc_common *powerpc = calloc(1, sizeof(struct powerpc_common));

	if (!powerpc)
		return ERROR_COMMAND_SYNTAX_ERROR;

	powerpc->common_magic = POWERPC_COMMON_MAGIC;
	powerpc->jtag_info.tap = target->tap;

	/* Hardware watchpoint */
	powerpc->dwp.avail = 2;
	powerpc->dwp.free = 2;
	powerpc->dwp.mask = 0x03;

	/* Hardware breakpoint */
	powerpc->hwb.avail = 8;
	powerpc->hwb.free = 8;
	powerpc->hwb.mask = 0xFF;

	powerpc->read_core_reg = powerpc_read_core_reg;
	powerpc->write_core_reg = powerpc_write_core_reg;

	target->arch_info = powerpc;

	/*
	 * Target will be stopped in debug mode
	 * as soon as target will be discovered.
	 */
	powerpc_write_debug_info_bits(target, DEBUG_HALT, 0);

	return ERROR_OK;
}



/*--------------------------------------------------------------------------*/

COMMAND_HANDLER(powerpc_setspr)
{
	struct target *target = get_current_target(CMD_CTX);
	struct powerpc_common *powerpc = target_to_powerpc(target);
	int retval;

	retval = powerpc_verify_pointer(CMD, powerpc);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* one or more argument, access a single register/select (write if third argument is given) */
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else {
		uint32_t spr_reg;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], spr_reg);
		/* Read SPR register */
		if (CMD_ARGC == 1) {
			uint32_t value;
			retval = powerpc_once_spr_read(target, spr_reg, &value);
			if (retval != ERROR_OK) {
				command_print(CMD, "couldn't access reg %" PRIi32, spr_reg);
				return ERROR_OK;
			}
			command_print(CMD, "spr reg %" PRIi32 " : 0x%8.8" PRIx32, spr_reg, value);
		/*Write a SPR register */
		} else if (CMD_ARGC == 2) {
			uint32_t value;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
			retval = powerpc_once_spr_write(target, spr_reg, value);
			if (retval != ERROR_OK) {
				command_print(CMD,
						"couldn't access spr reg %" PRIi32, spr_reg);
				return ERROR_OK;
			}
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(powerpc_ram_fill)
{
	struct target *target = get_current_target(CMD_CTX);
	struct powerpc_common *powerpc = target_to_powerpc(target);
	struct working_area *fill_algorithm;
	struct reg_param reg_params[5];
	struct powerpc_algorithm powerpc_info;
	int retval;
	int err;
	unsigned int i;
	struct cpu_status loc_cpu_status;

	uint32_t scratch_registers[POWERPC_NUM_CORE_REGS];
	uint8_t *memory_backup;

	uint32_t fill_value;

	err = ERROR_OK;

	/* Set arch info */
	powerpc_info.common_magic = POWERPC_COMMON_MAGIC;

	retval = powerpc_verify_pointer(CMD, powerpc);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* with only two parameters fill value = 0 */
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (CMD_ARGC == 2)
		fill_value = 0x0;
	else
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], fill_value);

	uint32_t start_address;
	uint32_t lenght;
	uint32_t end_address;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], start_address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], lenght);

	if ((lenght % 64) || (start_address & 0x3u))
	{
		command_print(CMD, "Start address must be 4 bytes aligned and length must multiple of 64 bytes ");
		return ERROR_OK;
	}

	end_address = start_address + lenght;

	if (((start_address >= target->working_area_phys) && (start_address < target->working_area_phys + target->working_area_size - 1)) || ((end_address >= target->working_area_phys) && (end_address < target->working_area_phys + target->working_area_size - 1)))
	{
		command_print(CMD, "ram_fill can't be used to fill RAM used as working area");
		return ERROR_OK;
	}

	/* Read scratch registers  and save them */
	for (i = 0; i < powerpc->core_cache->num_regs; i++) {
		scratch_registers[i] = buf_get_u32(
				powerpc->core_cache->reg_list[i].value,
				0,
				32);
	}

	/* RAM fill code */
	if (target_alloc_working_area(target, sizeof(spc58x_ram_fill),
			&fill_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do flash init step");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	memory_backup = (uint8_t *) malloc(fill_algorithm->size);
	powerpc_read_memory(target, fill_algorithm->address, 2, fill_algorithm->size / 2, memory_backup);

	err = target_write_buffer(target, fill_algorithm->address,
			sizeof(spc58x_ram_fill), (uint8_t *)spc58x_ram_fill);
	if (err != ERROR_OK) {
		target_free_working_area(target, fill_algorithm);
		return err;
	}

	/* start_address */
	init_reg_param(&reg_params[0], "r3", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, start_address);

	/* end_address */
	init_reg_param(&reg_params[1], "r4", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, end_address);

	/* fill value */
	init_reg_param(&reg_params[2], "r5", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, fill_value);


	/*
	 * Link register (in).
	 * Set link register to the breakpoint instruction at the end of the buffer.
	 * We use a software breakpoint to notify when done with algorithm execution.
	 */
	init_reg_param(&reg_params[3], "lr", 32, PARAM_IN);
	buf_set_u32(reg_params[3].value, 0, 32, fill_algorithm->address + (sizeof(spc58x_ram_fill) - 2));

	/*
	* Stack Pointer (in).
	*/
	init_reg_param(&reg_params[4], "r1", 32, PARAM_IN);
	buf_set_u32(reg_params[4].value, 0, 32, target->working_area_phys + target->working_area_size - 1);

	err = target_run_algorithm(target,
			0, NULL,
			4, reg_params,
			fill_algorithm->address, (fill_algorithm->address + fill_algorithm->size),
			5000, &powerpc_info);

	if (err != ERROR_OK)  {
		err = ERROR_TARGET_FAILURE;
	}

	powerpc_write_memory(target, fill_algorithm->address, 2, fill_algorithm->size / 2, memory_backup);

	/* Free resources */
	target_free_working_area(target, fill_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	/* Restore scratch registers */
	err = powerpc_once_read_cpu_scan_regs(target->tap, &loc_cpu_status);
	if (err != ERROR_OK)
		return err;

	/* Read core registers */
	for (i = 0; i < POWERPC_NUM_GP_REGS; i++) {
			err = powerpc_once_gpr_read(target, i, &powerpc->core_regs[i]);
			if (err != ERROR_OK)
				return err;
		}

	/* PC and MSR register already available in the cpu_info structure */
	powerpc->core_regs[PC] = loc_cpu_status.PC;
	powerpc->core_regs[MSR] = loc_cpu_status.MSR;

	err = powerpc_once_cr_read(target, &powerpc->core_regs[CR]);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_spr_read(target, SPR_LR, &powerpc->core_regs[LR]);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_spr_read(target, SPR_CTR, &powerpc->core_regs[CTR]);
	if (err != ERROR_OK)
		return err;

	err = powerpc_once_spr_read(target, SPR_XER, &powerpc->core_regs[XER]);
	if (err != ERROR_OK)
		return err;



	for (i = 0; i < powerpc->core_cache->num_regs; i++)
	{
		if (powerpc->core_regs[i] != scratch_registers[i])
		{
			/* Restore original context */
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
				powerpc->core_cache->reg_list[i].name,
				scratch_registers[i]);


			powerpc->core_cache->reg_list[i].valid = 1;
			powerpc->core_cache->reg_list[i].dirty = 1;
		}
		else
		{
			powerpc->core_cache->reg_list[i].valid = 1;
			powerpc->core_cache->reg_list[i].dirty = 0;
		}
		buf_set_u32(powerpc->core_cache->reg_list[i].value, 0, 32, scratch_registers[i]);
	}
	return err;
}

static const struct command_registration powerpc_exec_command_handlers[] = {
	{
		.name = "setspr",
		.handler = powerpc_setspr,
		.mode = COMMAND_EXEC,
		.usage = "regnum [value]",
		.help = "display/modify spr register",
	},
	{
		.name = "ram_fill",
		.handler = powerpc_ram_fill,
		.mode = COMMAND_EXEC,
		.usage = "regnum [value]",
		.help = "fill range of ram with 0x0",
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type powerpc_target = {
	.name = "powerpc",

	.poll = powerpc_poll,
	.arch_state = powerpc_arch_state,

	.target_request_data = powerpc_target_request_data,

	.halt = powerpc_halt,
	.resume = powerpc_resume,
	.step = powerpc_step,

	.assert_reset = powerpc_assert_reset,
	.deassert_reset = powerpc_deassert_reset,
	.soft_reset_halt = powerpc_soft_reset_halt,

	.get_gdb_reg_list = powerpc_get_gdb_reg_list,

	.read_memory = powerpc_read_memory,
	.write_memory = powerpc_write_memory,

	.run_algorithm = powerpc_run_algorithm,
	.start_algorithm = powerpc_start_algorithm,
	.wait_algorithm = powerpc_wait_algorithm,

	.add_breakpoint = powerpc_add_breakpoint,
	.remove_breakpoint = powerpc_remove_breakpoint,
	.add_watchpoint = powerpc_add_watchpoint,
	.remove_watchpoint = powerpc_remove_watchpoint,

	.target_create = powerpc_target_create,
	.init_target = powerpc_init_target,
	.examine = powerpc_examine,
	.deinit_target = powerpc_deinit_target,

	.commands = powerpc_exec_command_handlers,
};

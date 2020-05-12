/***************************************************************************
 *   Copyright (C) 2019 by STMicroelectronics                              *
 *                                                                         *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.                                        *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m33
	.thumb
	.thumb_func

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (64bit words)
 * r4 = flash base
 *
 * Clobbered:
 * r5   - rp
 * r6   - temp
 * r7   - wp
 */

#define STM32_FLASH_CR_OFFSET	0x28	/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x20	/* offset of SR register in FLASH struct */

wait_fifo:
	ldr		r7, [r0, #0]	/* read wp */
	cmp		r7, #0			/* abort if wp == 0 */
	beq		exit
	ldr		r5, [r0, #4]	/* read rp */
	subs	r6, r7, r5		/* number of bytes available for read in r6 */
	bmi		negative		/* if wrapped around, add size of buffer */
positive:
	cmp		r6, #8			/* wait until 8 bytes are available */
	bcc		wait_fifo

	movs	r6, #1			/* FLASH_CR = 1 (Set PROG bit) */
	str		r6, [r4, #STM32_FLASH_CR_OFFSET]
	ldmia	r5!, {r6}		/* read 1st word from src, increment ptr */
	stmia	r2!, {r6}		/* write 1st word to dest, increment ptr */
	dsb
	ldmia	r5!, {r6}		/* read 2nd word from src, increment ptr */
	stmia	r2!, {r6}		/* write 2nd word to dest, increment ptr */
	dsb
busy:
	ldr		r6, [r4, #STM32_FLASH_SR_OFFSET]
	lsrs	r6, #17			/* BSY (bit16) == 1 => operation in progress */
	bcs		busy			/* wait more... */
	ldr		r6, [r4, #STM32_FLASH_SR_OFFSET]
	uxtb	r6, r6			/* isolate error flags byte */
	cmp		r6, #0			/* PGSERR | SIZERR | PGAERR | WRPERR | PROGERR | OPERR */
	bne		error			/* fail... */

	cmp		r5, r1			/* wrap rp at end of buffer */
	bcs		superior
inferior:
	str		r5, [r0, #4]	/* store rp */
	subs	r3, r3, #1		/* decrement dword count */
	beq		exit			/* loop if not done */
	b		wait_fifo
negative:
	add		r6, r6, r1		/* add size of buffer */
	subs	r6, r0
	b		positive
superior:
	mov		r5, r0			/* wrap rp (rp = r0 +8)   */
	adds	r5, r5, #8		/* +8 to skip loader args */
	b		inferior
error:
	movs	r1, #0
	str		r1, [r0, #4]	/* set rp = 0 on error */
exit:
	mov		r0, r6			/* return status in r0 */
	bkpt	#0x00

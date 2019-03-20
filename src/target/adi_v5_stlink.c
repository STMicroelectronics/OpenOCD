/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 * Author(s): Antonio Borneo <borneo.antonio@gmail.com> for STMicroelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file
 * Utilities to support STMicroelectronics in-circuit debugger and programmer
 * "ST-Link" (only V2 and newer versions) to access ARM DAP through either
 * JTAG or ARM "Serial Wire Debug" (SWD).
 *
 * This implementation is complementary to the existing "hla" transport, which
 * is too much focused at Cortex-M, and supports also Cortex-A and SMP nodes.
 *
 * Single-DAP support only.
 *
 * for details, see "ARM IHI 0031A"
 * ARM Debug Interface v5 Architecture Specification
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

#include <transport/transport.h>
#include <jtag/interface.h>

#include <jtag/swd.h>

#define STLINK_DEBUG_PORT 0xffff

int stlink_dap_dap_read(unsigned short dap_port, unsigned short addr, uint32_t *val);
int stlink_dap_dap_write(unsigned short dap_port, unsigned short addr, uint32_t val);
int stlink_dap_ap_mem_read(struct adiv5_ap *ap, uint8_t *buffer,
	uint32_t size, uint32_t count, uint32_t address, bool addrinc);
int stlink_dap_ap_mem_write(struct adiv5_ap *ap, const uint8_t *buffer,
	uint32_t size, uint32_t count, uint32_t address, bool addrinc);

static int stlink_check_reconnect(struct adiv5_dap *dap);

static int stlink_swd_queue_dp_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	int retval = stlink_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = stlink_dap_dap_read(STLINK_DEBUG_PORT, reg, data);
	if (retval != ERROR_OK)
		dap->do_reconnect = true;
	return retval;
}

static int stlink_swd_queue_dp_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	int retval = stlink_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	/* ST-Link does not like that we set CORUNDETECT */
	if (reg == DP_CTRL_STAT)
		data &= ~CORUNDETECT;

	retval = stlink_dap_dap_write(STLINK_DEBUG_PORT, reg, data);
	if (retval != ERROR_OK)
		dap->do_reconnect = true;
	return retval;
}

static int stlink_swd_queue_ap_read(struct adiv5_ap *ap, unsigned reg,
		uint32_t *data)
{
	struct adiv5_dap *dap = ap->dap;
	int retval = stlink_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = stlink_dap_dap_read(ap->ap_num, reg, data);
	if (retval != ERROR_OK)
		dap->do_reconnect = true;
	return retval;
}

static int stlink_swd_queue_ap_write(struct adiv5_ap *ap, unsigned reg,
		uint32_t data)
{
	struct adiv5_dap *dap = ap->dap;
	int retval = stlink_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = stlink_dap_dap_write(ap->ap_num, reg, data);
	if (retval != ERROR_OK)
		dap->do_reconnect = true;
	return retval;
}

static int stlink_connect(struct adiv5_dap *dap)
{
	uint32_t dpidr;
	int retval;

	LOG_INFO("stlink_connect(%sconnect)", dap->do_reconnect ? "re" : "");

	dap->do_reconnect = false;
	dap_invalidate_cache(dap);

	retval = dap->ops->queue_dp_read(dap, DP_DPIDR, &dpidr);
	if (retval == ERROR_OK) {
		LOG_INFO("SWD DPIDR %#8.8" PRIx32, dpidr);
		retval = dap_dp_init(dap);
	} else
		dap->do_reconnect = true;

	return retval;
}

static int stlink_check_reconnect(struct adiv5_dap *dap)
{
	if (dap->do_reconnect)
		return stlink_connect(dap);

	return ERROR_OK;
}

static int stlink_swd_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	LOG_ERROR("stlink_swd_queue_ap_abort()");
	return ERROR_OK;
}

static int stlink_swd_run(struct adiv5_dap *dap)
{
	/* Here no LOG_DEBUG. This is called continuously! */

	/*
	 * ST-Link returns immediately after a DAP write, without waiting for it
	 * to complete.
	 * FIXME: Here we should check if the last operation is a read or a
	 * write, and issue the dummy read only to complete a write!
	 *
	 * Run a dummy read to DP_RDBUFF, as suggested in
	 * http://infocenter.arm.com/help/topic/com.arm.doc.faqs/ka16363.html
	 */
	return stlink_swd_queue_dp_read(dap, DP_RDBUFF, NULL);
}

#define stlink_ap_mem_read  stlink_dap_ap_mem_read
#define stlink_ap_mem_write stlink_dap_ap_mem_write

const struct dap_ops stlink_dap_swd_ops = {
	.connect = stlink_connect,
	.queue_dp_read = stlink_swd_queue_dp_read,
	.queue_dp_write = stlink_swd_queue_dp_write,
	.queue_ap_read = stlink_swd_queue_ap_read,
	.queue_ap_write = stlink_swd_queue_ap_write,
	.queue_ap_abort = stlink_swd_queue_ap_abort,
	.ap_mem_read = stlink_ap_mem_read,
	.ap_mem_write = stlink_ap_mem_write,
	.run = stlink_swd_run,
};

const struct dap_ops stlink_dap_jtag_ops = {
	.connect = stlink_connect,
	.queue_dp_read = stlink_swd_queue_dp_read,
	.queue_dp_write = stlink_swd_queue_dp_write,
	.queue_ap_read = stlink_swd_queue_ap_read,
	.queue_ap_write = stlink_swd_queue_ap_write,
	.queue_ap_abort = stlink_swd_queue_ap_abort,
	.ap_mem_read = stlink_ap_mem_read,
	.ap_mem_write = stlink_ap_mem_write,
	.run = stlink_swd_run,
};

static const struct command_registration stlink_commands[] = {
	{
		/*
		 * Set up SWD and JTAG targets identically, unless/until
		 * infrastructure improves ...  meanwhile, ignore all
		 * JTAG-specific stuff like IR length for SWD.
		 *
		 * REVISIT can we verify "just one SWD DAP" here/early?
		 */
		.name = "newdap",
		.jim_handler = jim_jtag_newtap,
		.mode = COMMAND_CONFIG,
		.help = "declare a new DAP"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stlink_handlers[] = {
	{
		.name = "stlink_dap",
		.mode = COMMAND_ANY,
		.help = "ST-Link command group",
		.chain = stlink_commands,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int stlink_jtag_select(struct command_context *ctx)
{
	LOG_DEBUG("stlink_jtag_select()");

	return register_commands(ctx, NULL, stlink_handlers);
}

static int stlink_swd_select(struct command_context *ctx)
{
	/* FIXME: only place where global 'jtag_interface' is still needed */
	extern struct jtag_interface *jtag_interface;
	const struct swd_driver *swd = jtag_interface->swd;
	int retval;

	LOG_DEBUG("stlink_swd_select()");

	retval = register_commands(ctx, NULL, stlink_handlers);
	if (retval != ERROR_OK)
		return retval;

	 /* be sure driver is in SWD mode; start
	  * with hardware default TRN (1), it can be changed later
	  */
	if (!swd || !swd->read_reg || !swd->write_reg || !swd->init) {
		LOG_DEBUG("no SWD driver?");
		return ERROR_FAIL;
	}

	retval = swd->init();
	if (retval != ERROR_OK) {
		LOG_DEBUG("can't init SWD driver");
		return retval;
	}

	return retval;
}

static int stlink_init(struct command_context *ctx)
{
	LOG_DEBUG("stlink_init()");

	adapter_deassert_reset();
	return ERROR_OK;
}

static struct transport stlink_jtag_transport = {
	.name = "stlink_jtag",
	.select = stlink_jtag_select,
	.init = stlink_init,
};

static struct transport stlink_swd_transport = {
	.name = "stlink_swd",
	.select = stlink_swd_select,
	.init = stlink_init,
};

static void stlink_constructor(void) __attribute__((constructor));
static void stlink_constructor(void)
{
	transport_register(&stlink_jtag_transport);
	transport_register(&stlink_swd_transport);
}

/** Returns true if the current debug session
 * is using SWD as its transport.
 */
bool transport_is_stlink_jtag(void)
{
	return get_current_transport() == &stlink_jtag_transport;
}

/** Returns true if the current debug session
 * is using SWD as its transport.
 */
bool transport_is_stlink_swd(void)
{
	return get_current_transport() == &stlink_swd_transport;
}

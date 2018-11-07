/***************************************************************************
 *   SWIM contributions by Ake Rehnman                                     *
 *   Copyright (C) 2017  Ake Rehnman                                       *
 *   ake.rehnman(at)gmail.com                                              *
 *                                                                         *
 *   Copyright (C) 2011-2012 by Mathias Kuester                            *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This code is based on https://github.com/texane/stlink                *
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

/* project specific includes */
#include <helper/bits.h>
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <jtag/swd.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include "libusb_common.h"

#define HLA_DEFAULT_APNUN     (0)

#define ENDPOINT_IN  0x80
#define ENDPOINT_OUT 0x00

#define STLINK_WRITE_TIMEOUT 1000
#define STLINK_READ_TIMEOUT 1000

#define STLINK_NULL_EP        0
#define STLINK_RX_EP          (1|ENDPOINT_IN)
#define STLINK_TX_EP          (2|ENDPOINT_OUT)
#define STLINK_TRACE_EP       (3|ENDPOINT_IN)

#define STLINK_V2_1_TX_EP     (1|ENDPOINT_OUT)
#define STLINK_V2_1_TRACE_EP  (2|ENDPOINT_IN)

#define STLINK_SG_SIZE        (31)
#define STLINK_DATA_SIZE      (4096)
#define STLINK_CMD_SIZE_V2    (16)
#define STLINK_CMD_SIZE_V1    (10)

#define STLINK_VID            (0x0483)
#define STLINK_V1_PID         (0x3744)
#define STLINK_V2_PID         (0x3748)
#define STLINK_V2_1_PID       (0x374B)
#define STLINK_V2_1_NO_MSD_PID  (0x3752)
#define STLINK_V3_DFU_PID       (0x374D)
#define STLINK_V3E_PID          (0x374E)
#define STLINK_V3S_PID          (0x374F)
#define STLINK_V3_2VCP_PID      (0x3753)

/* the current implementation of the stlink limits
 * 8bit read/writes to max 64 bytes.
 * The limit is 512 bytes with stlink v3 on usb 2.0 high speed */
/*
 * ST-Link/V1, ST-Link/V2 and ST-Link/V2.1 are full-speed USB devices and
 * this limits the bulk packet size and the 8bit read/writes to max 64 bytes.
 * ST-Link/V3 is a high speed USB 2.0 and the limit is 512 bytes.
 */
#define STLINKV2_MAX_RW8        (64)
#define STLINKV3_MAX_RW8        (512)

/*
 * ARM IHI 0031E: TAR Automatic address increment is only guaranteed to
 * operate on the 10 least significant bits of the address
 */
#define TAR_AUTOINCR_BLOCK      (1 << 10)

/* "WAIT" responses will be retried (with exponential backoff) at
 * most this many times before failing to caller.
 */
#define MAX_WAIT_RETRIES 8

enum stlink_jtag_api_version {
	STLINK_JTAG_API_V1 = 1,
	STLINK_JTAG_API_V2,
	STLINK_JTAG_API_V3,
};

/** */
struct stlink_usb_version {
	/** */
	int stlink;
	/** */
	int jtag;
	/** */
	int swim;
	/** highest supported jtag api version */
	enum stlink_jtag_api_version jtag_api_max;
};

/** */
struct stlink_usb_handle_s {
	/** */
	struct jtag_libusb_device_handle *fd;
	/** */
	struct libusb_transfer *trans;
	/** */
	uint8_t rx_ep;
	/** */
	uint8_t tx_ep;
	/** */
	uint8_t trace_ep;
	/** */
	uint8_t cmdbuf[STLINK_SG_SIZE];
	/** */
	uint8_t cmdidx;
	/** */
	uint8_t direction;
	/** */
	uint8_t databuf[STLINK_DATA_SIZE];
	/** */
	uint32_t max_mem_packet;
	/** */
	enum hl_transports transport;
	/** */
	struct stlink_usb_version version;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** this is the currently used jtag api */
	enum stlink_jtag_api_version jtag_api;
	/** */
	struct {
		/** whether SWO tracing is enabled or not */
		bool enabled;
		/** trace module source clock */
		uint32_t source_hz;
	} trace;
	/** reconnect is needed next time we try to query the
	 * status */
	bool reconnect_pending;
};

/* status codes */
#define STLINK_SWIM_ERR_OK             0x00
#define STLINK_SWIM_BUSY               0x01
#define STLINK_DEBUG_ERR_OK            0x80
#define STLINK_DEBUG_ERR_FAULT         0x81

#define STLINK_JTAG_SPI_ERROR                    0x02
#define STLINK_JTAG_DMA_ERROR                    0x03
#define STLINK_JTAG_UNKNOWN_JTAG_CHAIN           0x04
#define STLINK_JTAG_NO_DEVICE_CONNECTED          0x05
#define STLINK_JTAG_INTERNAL_ERROR               0x06
#define STLINK_JTAG_CMD_WAIT                     0x07
#define STLINK_JTAG_CMD_ERROR                    0x08
#define STLINK_JTAG_GET_IDCODE_ERROR             0x09
#define STLINK_JTAG_ALIGNMENT_ERROR              0x0A
#define STLINK_JTAG_DBG_POWER_ERROR              0x0B
#define STLINK_JTAG_WRITE_ERROR                  0x0C
#define STLINK_JTAG_WRITE_VERIF_ERROR            0x0D
#define STLINK_JTAG_ALREADY_OPENED_IN_OTHER_MODE 0x0E

#define STLINK_SWD_AP_WAIT             0x10
#define STLINK_SWD_AP_FAULT            0x11
#define STLINK_SWD_AP_ERROR            0x12
#define STLINK_SWD_AP_PARITY_ERROR     0x13
#define STLINK_SWD_DP_WAIT             0x14
#define STLINK_SWD_DP_FAULT            0x15
#define STLINK_SWD_DP_ERROR            0x16
#define STLINK_SWD_DP_PARITY_ERROR     0x17
#define STLINK_SWD_AP_WDATA_ERROR      0x18
#define STLINK_SWD_AP_STICKY_ERROR     0x19
#define STLINK_SWD_AP_STICKYORUN_ERROR 0x1A
#define STLINK_AP_ALREADY_USED         0x1B
#define STLINK_TRACE_AP_TURNAROUND     0x1C
#define STLINK_BAD_AP                  0x1D
#define STLINK_SWV_NOT_AVAILABLE       0x20
#define STLINK_NO_JUMP_TO_USB_LOADER   0x21
#define STLINK_JTAG_TCPID_NOT_FOUND    0x30
#define STLINK_JTAG_TCPID_MAX_REACHED  0x31

#define STLINK_CORE_RUNNING            0x80
#define STLINK_CORE_HALTED             0x81
#define STLINK_CORE_STAT_UNKNOWN       -1

/* stlink commands */
#define STLINK_GET_VERSION             0xF1
#define STLINK_DEBUG_COMMAND           0xF2
#define STLINK_DFU_COMMAND             0xF3
#define STLINK_SWIM_COMMAND            0xF4
#define STLINK_GET_CURRENT_MODE        0xF5
#define STLINK_GET_TARGET_VOLTAGE      0xF7
#define STLINK_APIV3_GET_VERSION_EX    0xFB

#define STLINK_DEV_DFU_MODE            0x00
#define STLINK_DEV_MASS_MODE           0x01
#define STLINK_DEV_DEBUG_MODE          0x02
#define STLINK_DEV_SWIM_MODE           0x03
#define STLINK_DEV_BOOTLOADER_MODE     0x04
#define STLINK_DEV_UNKNOWN_MODE        -1

#define STLINK_DFU_EXIT                0x07

/*
	STLINK_SWIM_ENTER_SEQ
	1.3ms low then 750Hz then 1.5kHz

	STLINK_SWIM_GEN_RST
	STM8 DM pulls reset pin low 50us

	STLINK_SWIM_SPEED
	uint8_t (0=low|1=high)

	STLINK_SWIM_WRITEMEM
	uint16_t length
	uint32_t address

	STLINK_SWIM_RESET
	send syncronization seq (16us low, response 64 clocks low)
*/
#define STLINK_SWIM_ENTER                  0x00
#define STLINK_SWIM_EXIT                   0x01
#define STLINK_SWIM_READ_CAP               0x02
#define STLINK_SWIM_SPEED                  0x03
#define STLINK_SWIM_ENTER_SEQ              0x04
#define STLINK_SWIM_GEN_RST                0x05
#define STLINK_SWIM_RESET                  0x06
#define STLINK_SWIM_ASSERT_RESET           0x07
#define STLINK_SWIM_DEASSERT_RESET         0x08
#define STLINK_SWIM_READSTATUS             0x09
#define STLINK_SWIM_WRITEMEM               0x0a
#define STLINK_SWIM_READMEM                0x0b
#define STLINK_SWIM_READBUF                0x0c

#define STLINK_DEBUG_GETSTATUS             0x01
#define STLINK_DEBUG_FORCEDEBUG            0x02
#define STLINK_DEBUG_APIV1_RESETSYS        0x03
#define STLINK_DEBUG_APIV1_READALLREGS     0x04
#define STLINK_DEBUG_APIV1_READREG         0x05
#define STLINK_DEBUG_APIV1_WRITEREG        0x06
#define STLINK_DEBUG_READMEM_32BIT         0x07
#define STLINK_DEBUG_WRITEMEM_32BIT        0x08
#define STLINK_DEBUG_RUNCORE               0x09
#define STLINK_DEBUG_STEPCORE              0x0a
#define STLINK_DEBUG_APIV1_SETFP           0x0b
#define STLINK_DEBUG_READMEM_8BIT          0x0c
#define STLINK_DEBUG_WRITEMEM_8BIT         0x0d
#define STLINK_DEBUG_APIV1_CLEARFP         0x0e
#define STLINK_DEBUG_APIV1_WRITEDEBUGREG   0x0f
#define STLINK_DEBUG_APIV1_SETWATCHPOINT   0x10

#define STLINK_DEBUG_ENTER_JTAG            0x00
#define STLINK_DEBUG_ENTER_SWD             0xa3
#define STLINK_ENTER_JTAG_NO_CORE_RESET    0xA4

#define STLINK_DEBUG_APIV1_ENTER           0x20
#define STLINK_DEBUG_EXIT                  0x21
#define STLINK_DEBUG_READCOREID            0x22

#define STLINK_DEBUG_APIV2_ENTER           0x30
#define STLINK_DEBUG_APIV2_READ_IDCODES    0x31
#define STLINK_DEBUG_APIV2_RESETSYS        0x32
#define STLINK_DEBUG_APIV2_READREG         0x33
#define STLINK_DEBUG_APIV2_WRITEREG        0x34
#define STLINK_DEBUG_APIV2_WRITEDEBUGREG   0x35
#define STLINK_DEBUG_APIV2_READDEBUGREG    0x36
#define STLINK_DEBUG_APIV2_SETWATCHPOINT2  0x37
#define STLINK_DEBUG_APIV2_SETFP2          0x38
#define STLINK_DEBUG_APIV2_CLEARFP2        0x39
#define STLINK_DEBUG_APIV2_READALLREGS     0x3A
#define STLINK_DEBUG_APIV2_GETLASTRWSTATUS 0x3B
#define STLINK_DEBUG_APIV2_DRIVE_NRST      0x3C
#define STLINK_DEBUG_APIV2_READFPUREGS     0x3D
#define STLINK_DEBUG_APIV3_GETLASTRWSTATUS 0x3E

#define STLINK_DEBUG_APIV2_START_TRACE_RX  0x40
#define STLINK_DEBUG_APIV2_STOP_TRACE_RX   0x41
#define STLINK_DEBUG_APIV2_GET_TRACE_NB    0x42
#define STLINK_DEBUG_APIV2_SWD_SET_FREQ    0x43
#define STLINK_DEBUG_APIV2_JTAG_SET_FREQ   0x44
#define STLINK_JTAG_READ_DAP_REG           0x45
#define STLINK_JTAG_WRITE_DAP_REG          0x46
#define STLINK_DEBUG_APIV2_READMEM_16BIT   0x47
#define STLINK_DEBUG_APIV2_WRITEMEM_16BIT  0x48
#define STLINK_DEBUG_APIV2_BLINK_LED       0x49
#define STLINK_DEBUG_APIV2_GET_DISK_NAME   0x4A
#define STLINK_DEBUG_APIV2_INIT_AP         0x4B
#define STLINK_DEBUG_APIV2_CLOSE_AP_DBG    0x4C

#define STLINK_DEBUG_WRITEMEM_32BIT_NO_ADDR_INC         0x50

#define STLINK_DEBUG_READMEM_32BIT_NO_ADDR_INC          0x54

#define STLINK_APIV3_SET_COM_FREQ          0x61
#define STLINK_APIV3_GET_COM_FREQ          0x62
#define STLINK_APIV3_SWITCH_STLINK_FREQ    0x63

/* parameters */
#define STLINK_DEBUG_APIV2_DRIVE_NRST_LOW   0x00
#define STLINK_DEBUG_APIV2_DRIVE_NRST_HIGH  0x01
#define STLINK_DEBUG_APIV2_DRIVE_NRST_PULSE 0x02

#define STLINK_DEBUG_FLASHPATCH_LOWER       0
#define STLINK_DEBUG_FLASHPATCH_UPPER       1
#define STLINK_DEBUG_FLASHPATCH_ALL         2

#define JTAG_AP_CORTEXM_CORE                1
#define JTAG_AP_NO_CORE                     0

#define STLINK_TRACE_SIZE               4096
#define STLINK_TRACE_MAX_HZ             2000000
#define STLINK_TRACE_MIN_VERSION        13

#define STLINK_V3_MAX_FREQ_NB               10

/** */
enum stlink_mode {
	STLINK_MODE_UNKNOWN = 0,
	STLINK_MODE_DFU,
	STLINK_MODE_MASS,
	STLINK_MODE_DEBUG_JTAG,
	STLINK_MODE_DEBUG_SWD,
	STLINK_MODE_DEBUG_SWIM
};

#define REQUEST_SENSE        0x03
#define REQUEST_SENSE_LENGTH 18

struct speed_map {
	int speed;
	int speed_divisor;
};

/* SWD clock speed */
static const struct speed_map stlink_khz_to_speed_map[] = {
	{4000, 0},
	{1800, 1}, /* default */
	{1200, 2},
	{950,  3},
	{480,  7},
	{240, 15},
	{125, 31},
	{100, 40},
	{50,  79},
	{25, 158},
	{15, 265},
	{5,  798}
};

/* JTAG clock speed */
static const struct speed_map stlink_khz_to_speed_map_jtag[] = {
	{18000, 2},
	{9000,  4},
	{4500,  8},
	{2250, 16},
	{1125, 32}, /* default */
	{562,  64},
	{281, 128},
	{140, 256}
};

static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size);
static int stlink_swim_status(void *handle);
static int stlink_usb_reset(void *handle);


/** */
__attribute__((unused))
static unsigned int stlink_usb_block(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.stlink == 3)
		return STLINKV3_MAX_RW8;
	else
		return STLINKV2_MAX_RW8;
}


/** */
static int stlink_usb_xfer_v1_get_status(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* read status */
	memset(h->cmdbuf, 0, STLINK_SG_SIZE);

	if (jtag_libusb_bulk_read(h->fd, h->rx_ep, (char *)h->cmdbuf,
			13, STLINK_READ_TIMEOUT) != 13)
		return ERROR_FAIL;

	uint32_t t1;

	t1 = buf_get_u32(h->cmdbuf, 0, 32);

	/* check for USBS */
	if (t1 != 0x53425355)
		return ERROR_FAIL;
	/*
	 * CSW status:
	 * 0 success
	 * 1 command failure
	 * 2 phase error
	 */
	if (h->cmdbuf[12] != 0)
		return ERROR_FAIL;

	return ERROR_OK;
}

/** */
static int stlink_usb_xfer_rw(void *handle, int cmdsize, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (jtag_libusb_bulk_write(h->fd, h->tx_ep, (char *)h->cmdbuf, cmdsize,
			STLINK_WRITE_TIMEOUT) != cmdsize) {
		return ERROR_FAIL;
	}

	if (h->direction == h->tx_ep && size) {
		if (jtag_libusb_bulk_write(h->fd, h->tx_ep, (char *)buf,
				size, STLINK_WRITE_TIMEOUT) != size) {
			LOG_DEBUG("bulk write failed");
			return ERROR_FAIL;
		}
	} else if (h->direction == h->rx_ep && size) {
		if (jtag_libusb_bulk_read(h->fd, h->rx_ep, (char *)buf,
				size, STLINK_READ_TIMEOUT) != size) {
			LOG_DEBUG("bulk read failed");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_xfer_v1_get_sense(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 16);

	h->cmdbuf[h->cmdidx++] = REQUEST_SENSE;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = REQUEST_SENSE_LENGTH;

	res = stlink_usb_xfer_rw(handle, REQUEST_SENSE_LENGTH, h->databuf, 16);

	if (res != ERROR_OK)
		return res;

	if (stlink_usb_xfer_v1_get_status(handle) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/*
	transfers block in cmdbuf
	<size> indicates number of bytes in the following
	data phase.
*/
static int stlink_usb_xfer(void *handle, const uint8_t *buf, int size)
{
	int err, cmdsize = STLINK_CMD_SIZE_V2;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.stlink == 1) {
		cmdsize = STLINK_SG_SIZE;
		/* put length in bCBWCBLength */
		h->cmdbuf[14] = h->cmdidx-15;
	}

	err = stlink_usb_xfer_rw(handle, cmdsize, buf, size);

	if (err != ERROR_OK)
		return err;

	if (h->version.stlink == 1) {
		if (stlink_usb_xfer_v1_get_status(handle) != ERROR_OK) {
			/* check csw status */
			if (h->cmdbuf[12] == 1) {
				LOG_DEBUG("get sense");
				if (stlink_usb_xfer_v1_get_sense(handle) != ERROR_OK)
					return ERROR_FAIL;
			}
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stlink_dap_arp_init);
/**
    Converts an STLINK status code held in the first byte of a response
    to an openocd error, logs any error/wait status as debug output.
*/
static int stlink_usb_error_check(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->transport == HL_TRANSPORT_SWIM) {
		switch (h->databuf[0]) {
			case STLINK_SWIM_ERR_OK:
				return ERROR_OK;
			case STLINK_SWIM_BUSY:
				return ERROR_WAIT;
			default:
				LOG_DEBUG("unknown/unexpected STLINK status code 0x%x", h->databuf[0]);
				return ERROR_FAIL;
		}
	}

	/* TODO: no error checking yet on api V1 */
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->databuf[0] = STLINK_DEBUG_ERR_OK;

	switch (h->databuf[0]) {
		case STLINK_DEBUG_ERR_OK:
			return ERROR_OK;
		case STLINK_DEBUG_ERR_FAULT:
			LOG_DEBUG("SWD fault response (0x%x)", STLINK_DEBUG_ERR_FAULT);
			return ERROR_FAIL;
		case STLINK_SWD_AP_WAIT:
			LOG_DEBUG("wait status SWD_AP_WAIT (0x%x)", STLINK_SWD_AP_WAIT);
			return ERROR_WAIT;
		case STLINK_SWD_DP_WAIT:
			LOG_DEBUG("wait status SWD_DP_WAIT (0x%x)", STLINK_SWD_DP_WAIT);
			return ERROR_WAIT;
		case STLINK_JTAG_SPI_ERROR:
			LOG_DEBUG("JTAG_INTERNAL_ERROR (SPI)");
			return ERROR_FAIL;
		case STLINK_JTAG_DMA_ERROR:
			LOG_DEBUG("JTAG_INTERNAL_ERROR (DMA)");
			return ERROR_FAIL;
		case STLINK_JTAG_UNKNOWN_JTAG_CHAIN:
			LOG_DEBUG("UNKNOWN_JTAG_CHAIN");
			return ERROR_FAIL;
		case STLINK_JTAG_NO_DEVICE_CONNECTED:
			LOG_DEBUG("NO_DEVICE_CONNECTED");
			return ERROR_FAIL;
		case STLINK_JTAG_INTERNAL_ERROR:
			LOG_DEBUG("JTAG_INTERNAL_ERROR");
			return ERROR_FAIL;
		case STLINK_JTAG_CMD_WAIT:
			LOG_DEBUG("wait status STLINK_JTAG_CMD_WAIT");
			return ERROR_WAIT;
		case STLINK_JTAG_CMD_ERROR:
			LOG_DEBUG("JTAG_CMD_ERROR");
			return ERROR_FAIL;
		case STLINK_JTAG_GET_IDCODE_ERROR:
			LOG_DEBUG("JTAG_GET_IDCODE_ERROR");
			return ERROR_FAIL;
		case STLINK_JTAG_ALIGNMENT_ERROR:
			LOG_DEBUG("JTAG_ALIGNMENT_ERROR");
			return ERROR_FAIL;
		case STLINK_JTAG_DBG_POWER_ERROR:
			LOG_DEBUG("JTAG_DBG_POWER_ERROR");
			return ERROR_FAIL;
		case STLINK_JTAG_WRITE_ERROR:
			LOG_DEBUG("Write error");
			return ERROR_FAIL;
		case STLINK_JTAG_WRITE_VERIF_ERROR:
			LOG_DEBUG("Write verify error, ignoring");
			return ERROR_OK;
		case STLINK_SWD_AP_FAULT:
			/* git://git.ac6.fr/openocd commit 657e3e885b9ee10
			 * returns ERROR_OK with the comment:
			 * Change in error status when reading outside RAM.
			 * This fix allows CDT plugin to visualize memory.
			 */
			LOG_DEBUG("STLINK_SWD_AP_FAULT");
			return ERROR_OK;
		case STLINK_SWD_AP_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_AP_PARITY_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_PARITY_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_DP_FAULT:
			LOG_DEBUG("STLINK_SWD_DP_FAULT");
			return ERROR_FAIL;
		case STLINK_SWD_DP_ERROR:
			LOG_DEBUG("STLINK_SWD_DP_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_DP_PARITY_ERROR:
			LOG_DEBUG("STLINK_SWD_DP_PARITY_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_AP_WDATA_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_WDATA_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_AP_STICKY_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_STICKY_ERROR");
			/* Seen when reading address out of range (0xFFFFFFFF) */
			/* Seems usb_reset helps the following r/w accesses */
			// stlink_usb_reset(h);
			stlink_dap_arp_init(NULL);
			return ERROR_FAIL;
		case STLINK_SWD_AP_STICKYORUN_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_STICKYORUN_ERROR");
			return ERROR_FAIL;
		case STLINK_JTAG_ALREADY_OPENED_IN_OTHER_MODE:
			LOG_DEBUG("JTAG_ALREADY_OPENED_IN_OTHER_MODE");
			return ERROR_FAIL;
		case STLINK_AP_ALREADY_USED:
			LOG_DEBUG("AP_ALREADY_USED");
			return ERROR_FAIL;
		case STLINK_TRACE_AP_TURNAROUND:
			LOG_DEBUG("TRACE_AP_TURNAROUND");
			return ERROR_FAIL;
		case STLINK_BAD_AP:
			LOG_DEBUG("BAD_AP");
			return ERROR_FAIL;
		default:
			LOG_DEBUG("unknown/unexpected STLINK status code 0x%x", h->databuf[0]);
			return ERROR_FAIL;
	}
}


/** Issue an STLINK command via USB transfer, with retries on any wait status responses.

    Works for commands where the STLINK_DEBUG status is returned in the first
    byte of the response packet. For SWIM a SWIM_READSTATUS is requested instead.

    Returns an openocd result code.
*/
static int stlink_cmd_allow_retry(void *handle, const uint8_t *buf, int size)
{
	int retries = 0;
	int res;
	struct stlink_usb_handle_s *h = handle;

	while (1) {
		if ((h->transport != HL_TRANSPORT_SWIM) || !retries) {
			res = stlink_usb_xfer(handle, buf, size);
			if (res != ERROR_OK)
				return res;
		}

		if (h->transport == HL_TRANSPORT_SWIM) {
			res = stlink_swim_status(handle);
			if (res != ERROR_OK)
				return res;
		}

		res = stlink_usb_error_check(handle);
		if (res == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			usleep((1<<retries++) * 1000);
			continue;
		}
		return res;
	}
}

/** */
static int stlink_usb_read_trace(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	assert(h->version.stlink >= 2);

	if (jtag_libusb_bulk_read(h->fd, h->trace_ep, (char *)buf,
			size, STLINK_READ_TIMEOUT) != size) {
		LOG_ERROR("bulk trace read failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/*
	this function writes transfer length in
	the right place in the cb
*/
static void stlink_usb_set_cbw_transfer_datalength(void *handle, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	buf_set_u32(h->cmdbuf+8, 0, 32, size);
}

static void stlink_usb_xfer_v1_create_cmd(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	/* fill the send buffer */
	strcpy((char *)h->cmdbuf, "USBC");
	h->cmdidx += 4;
	/* csw tag not used */
	buf_set_u32(h->cmdbuf+h->cmdidx, 0, 32, 0);
	h->cmdidx += 4;
	/* cbw data transfer length (in the following data phase in or out) */
	buf_set_u32(h->cmdbuf+h->cmdidx, 0, 32, size);
	h->cmdidx += 4;
	/* cbw flags */
	h->cmdbuf[h->cmdidx++] = (direction == h->rx_ep ? ENDPOINT_IN : ENDPOINT_OUT);
	h->cmdbuf[h->cmdidx++] = 0; /* lun */
	/* cdb clength (is filled in at xfer) */
	h->cmdbuf[h->cmdidx++] = 0;
}

/** */
static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	h->direction = direction;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, STLINK_SG_SIZE);
	memset(h->databuf, 0, STLINK_DATA_SIZE);

	if (h->version.stlink == 1)
		stlink_usb_xfer_v1_create_cmd(handle, direction, size);
}

/** */
static int stlink_usb_version(void *handle)
{
	int res;
	uint16_t v;
	uint8_t m_version = 0;
	uint8_t bridge = 0;
	char v_str[5 * (1 + 3) + 1]; /* VvJjMmBbSs */
	char *p;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* FIXME: don't use PID, but first old command to get V3, then new command */
	if (h->pid == STLINK_V3E_PID || h->pid == STLINK_V3S_PID || h->pid == STLINK_V3_2VCP_PID) {
		stlink_usb_init_buffer(handle, h->rx_ep, 16);
		h->cmdbuf[h->cmdidx++] = STLINK_APIV3_GET_VERSION_EX;
		res = stlink_usb_xfer(handle, h->databuf, 12);

		if (res != ERROR_OK)
			return res;

		h->version.stlink = h->databuf[0];
		h->version.swim = h->databuf[1];
		h->version.jtag = h->databuf[2];
		m_version = h->databuf[3];
		bridge = h->databuf[4];

		h->vid = (h->databuf[9] << 8) | h->databuf[8];
		h->pid = (h->databuf[11] << 8) | h->databuf[10];

		h->version.jtag_api_max = STLINK_JTAG_API_V3;
	} else {
		stlink_usb_init_buffer(handle, h->rx_ep, 6);
		h->cmdbuf[h->cmdidx++] = STLINK_GET_VERSION;

		res = stlink_usb_xfer(handle, h->databuf, 6);

		if (res != ERROR_OK)
			return res;

		v = (h->databuf[0] << 8) | h->databuf[1];

		h->version.stlink = (v >> 12) & 0x0f;
		h->version.jtag = (v >> 6) & 0x3f;
		h->version.swim = v & 0x3f;
		h->vid = buf_get_u32(h->databuf, 16, 16);
		h->pid = buf_get_u32(h->databuf, 32, 16);

		/* set the supported jtag api version
		 * API V2 is supported since JTAG V11
		 */
		if (h->version.jtag >= 11)
			h->version.jtag_api_max = STLINK_JTAG_API_V2;
		else
			h->version.jtag_api_max = STLINK_JTAG_API_V1;
	}

	p = v_str;
	p += sprintf(p, "V%d", h->version.stlink);
	if (h->version.jtag || !m_version)
		p += sprintf(p, "J%d", h->version.jtag);
	if (m_version)
		p += sprintf(p, "M%d", m_version);
	if (bridge)
		p += sprintf(p, "B%d", bridge);
	if (h->version.swim || !m_version)
		p += sprintf(p, "S%d", h->version.swim);

	LOG_INFO("STLINK %s (API v%d) VID:PID %04X:%04X",
		v_str,
		h->version.jtag_api_max,
		h->vid,
		h->pid);

	return ERROR_OK;
}

static int stlink_usb_check_voltage(void *handle, float *target_voltage)
{
	struct stlink_usb_handle_s *h = handle;
	uint32_t adc_results[2];

	/* only supported by stlink v2/v3 and for firmware >= 13 */
	if (h->version.stlink == 1 || (h->version.stlink == 2 && h->version.jtag < 13))
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 8);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_TARGET_VOLTAGE;

	int result = stlink_usb_xfer(handle, h->databuf, 8);

	if (result != ERROR_OK)
		return result;

	/* convert result */
	adc_results[0] = le_to_h_u32(h->databuf);
	adc_results[1] = le_to_h_u32(h->databuf + 4);

	*target_voltage = 0;

	if (adc_results[0])
		*target_voltage = 2 * ((float)adc_results[1]) * (float)(1.2 / adc_results[0]);

	LOG_INFO("Target voltage: %f", (double)*target_voltage);

	return ERROR_OK;
}

static int stlink_usb_set_swdclk(void *handle, uint16_t clk_divisor)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* only supported by stlink/v2 and for firmware >= 22 */
	if (h->version.stlink == 1 || h->version.stlink == 3 || \
		(h->version.stlink == 2 && h->version.jtag < 22))
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_SWD_SET_FREQ;
	h_u16_to_le(h->cmdbuf+h->cmdidx, clk_divisor);
	h->cmdidx += 2;

	int result = stlink_cmd_allow_retry(handle, h->databuf, 2);

	if (result != ERROR_OK)
		return result;

	return ERROR_OK;
}

static int stlink_usb_set_jtagclk(void *handle, uint16_t clk_divisor)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* only supported by stlink/v2 and for firmware >= 24 */
	if (h->version.stlink == 1 || h->version.stlink == 3 || \
		(h->version.stlink == 2 && h->version.jtag < 24))
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_JTAG_SET_FREQ;
	h_u16_to_le(h->cmdbuf+h->cmdidx, clk_divisor);
	h->cmdidx += 2;

	int result = stlink_cmd_allow_retry(handle, h->databuf, 2);

	if (result != ERROR_OK)
		return result;

	return ERROR_OK;
}

/** */
static int stlink_usb_current_mode(void *handle, uint8_t *mode)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_CURRENT_MODE;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	*mode = h->databuf[0];

	return ERROR_OK;
}

/** */
static int stlink_usb_mode_enter(void *handle, enum stlink_mode type)
{
	int rx_size = 0;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* on api V2 we are able the read the latest command
	 * status
	 * TODO: we need the test on api V1 too
	 */
	if (h->jtag_api == STLINK_JTAG_API_V2 || h->jtag_api == STLINK_JTAG_API_V3)
		rx_size = 2;

	stlink_usb_init_buffer(handle, h->rx_ep, rx_size);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
			LOG_DEBUG("stlink_usb_mode_enter(JTAG)");
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			if (h->jtag_api == STLINK_JTAG_API_V1)
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_ENTER;
			h->cmdbuf[h->cmdidx++] = STLINK_ENTER_JTAG_NO_CORE_RESET;
			break;
		case STLINK_MODE_DEBUG_SWD:
			LOG_DEBUG("stlink_usb_mode_enter(SWD)");
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			if (h->jtag_api == STLINK_JTAG_API_V1)
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_ENTER;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_ENTER_SWD;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			LOG_DEBUG("stlink_usb_mode_enter(SWIM)");
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_ENTER;
			/* no answer for this function... */
			rx_size = 0;
			break;
		case STLINK_MODE_DFU:
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	return stlink_cmd_allow_retry(handle, h->databuf, rx_size);
}

/** */
static int stlink_usb_mode_leave(void *handle, enum stlink_mode type)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, STLINK_NULL_EP, 0);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
		case STLINK_MODE_DEBUG_SWD:
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_EXIT;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_EXIT;
			break;
		case STLINK_MODE_DFU:
			h->cmdbuf[h->cmdidx++] = STLINK_DFU_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_DFU_EXIT;
			break;
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	res = stlink_usb_xfer(handle, 0, 0);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

static int stlink_usb_assert_srst(void *handle, int srst);

static enum stlink_mode stlink_get_mode(enum hl_transports t)
{
	switch (t) {
	case HL_TRANSPORT_SWD:
		return STLINK_MODE_DEBUG_SWD;
	case HL_TRANSPORT_JTAG:
		return STLINK_MODE_DEBUG_JTAG;
	case HL_TRANSPORT_SWIM:
		return STLINK_MODE_DEBUG_SWIM;
	default:
		return STLINK_MODE_UNKNOWN;
	}
}

/** */
static int stlink_exit_mode (void *handle)
{
	int res;
	uint8_t mode;
	enum stlink_mode emode;

	assert(handle != NULL);

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* try to exit current mode */
	switch (mode) {
		case STLINK_DEV_DFU_MODE:
			emode = STLINK_MODE_DFU;
			break;
		case STLINK_DEV_DEBUG_MODE:
			emode = STLINK_MODE_DEBUG_SWD;
			break;
		case STLINK_DEV_SWIM_MODE:
			emode = STLINK_MODE_DEBUG_SWIM;
			break;
		case STLINK_DEV_BOOTLOADER_MODE:
		case STLINK_DEV_MASS_MODE:
		default:
			emode = STLINK_MODE_UNKNOWN;
			break;
	}

	if (emode != STLINK_MODE_UNKNOWN) {
		LOG_DEBUG("E-MODE: 0x%02X", emode);
		res = stlink_usb_mode_leave(handle, emode);
		if (res != ERROR_OK)
			return res;
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_init_mode(void *handle, bool connect_under_reset)
{
	int res;
	uint8_t mode;
	enum stlink_mode emode;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	res = stlink_exit_mode(handle);
	if (res != ERROR_OK)
		return res;

	res = stlink_usb_current_mode(handle, &mode);
	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* we check the target voltage here as an aid to debugging connection problems.
	 * the stlink requires the target Vdd to be connected for reliable debugging.
	 * this cmd is supported in all modes except DFU
	 */
	if (mode != STLINK_DEV_DFU_MODE) {

		float target_voltage;

		/* check target voltage (if supported) */
		res = stlink_usb_check_voltage(h, &target_voltage);

		if (res != ERROR_OK) {
			if (res != ERROR_COMMAND_NOTFOUND)
				LOG_ERROR("voltage check failed");
			/* attempt to continue as it is not a catastrophic failure */
		} else {
			/* check for a sensible target voltage, operating range is 1.65-5.5v
			 * according to datasheet */
			if (target_voltage < 1.5)
				LOG_ERROR("target voltage may be too low for reliable debugging");
		}
	}

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* set selected mode */
	emode = stlink_get_mode(h->transport);

	if (emode == STLINK_MODE_UNKNOWN) {
		LOG_ERROR("selected mode (transport) not supported");
		return ERROR_FAIL;
	}

	/* preliminary SRST assert:
	 * We want SRST is asserted before activating debug signals (mode_enter).
	 * As the required mode has not been set, the adapter may not know what pin to use.
	 * Tested firmware STLINK v2 JTAG v29 API v2 SWIM v0 uses T_NRST pin by default
	 * Tested firmware STLINK v2 JTAG v27 API v2 SWIM v6 uses T_NRST pin by default
	 * after power on, SWIM_RST stays unchanged */
	if (connect_under_reset && emode != STLINK_MODE_DEBUG_SWIM)
		stlink_usb_assert_srst(handle, 0);
		/* do not check the return status here, we will
		   proceed and enter the desired mode below
		   and try asserting srst again. */

	res = stlink_usb_mode_enter(handle, emode);
	if (res != ERROR_OK)
		return res;

	/* assert SRST again: a little bit late but now the adapter knows for sure what pin to use */
	if (h->transport == HL_TRANSPORT_SWIM && connect_under_reset) {
		res = stlink_usb_assert_srst(handle, 0);
		if (res != ERROR_OK)
			return res;
	}

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: 0x%02X", mode);

	return ERROR_OK;
}

/* request status from last swim request */
static int stlink_swim_status(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 4);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READSTATUS;
	res = stlink_usb_xfer(handle, h->databuf, 4);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}
/*
	the purpose of this function is unknown...
	capabilites? anyway for swim v6 it returns
	0001020600000000
*/
__attribute__((unused))
static int stlink_swim_cap(void *handle, uint8_t *cap)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 8);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READ_CAP;
	h->cmdbuf[h->cmdidx++] = 0x01;
	res = stlink_usb_xfer(handle, h->databuf, 8);
	if (res != ERROR_OK)
		return res;
	memcpy(cap, h->databuf, 8);
	return ERROR_OK;
}

/*	debug dongle assert/deassert sreset line */
static int stlink_swim_assert_reset(void *handle, int reset)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	if (!reset)
		h->cmdbuf[h->cmdidx++] = STLINK_SWIM_ASSERT_RESET;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_SWIM_DEASSERT_RESET;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*
	send swim enter seq
	1.3ms low then 750Hz then 1.5kHz
*/
static int stlink_swim_enter(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_ENTER_SEQ;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*	switch high/low speed swim */
static int stlink_swim_speed(void *handle, int speed)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_SPEED;
	if (speed)
		h->cmdbuf[h->cmdidx++] = 1;
	else
		h->cmdbuf[h->cmdidx++] = 0;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*
	initiate srst from swim.
	nrst is pulled low for 50us.
*/
static int stlink_swim_generate_rst(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_GEN_RST;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*
	send resyncronize sequence
	swim is pulled low for 16us
	reply is 64 clks low
*/
static int stlink_swim_resync(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_RESET;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

static int stlink_swim_writebytes(void *handle, uint32_t addr, uint32_t len, const uint8_t *data)
{
	struct stlink_usb_handle_s *h = handle;
	int res;
	unsigned int i;
	unsigned int datalen = 0;
	int cmdsize = STLINK_CMD_SIZE_V2;

	if (len > STLINK_DATA_SIZE)
		return ERROR_FAIL;

	if (h->version.stlink == 1)
		cmdsize = STLINK_SG_SIZE;

	stlink_usb_init_buffer(handle, h->tx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_WRITEMEM;
	h_u16_to_be(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h_u32_to_be(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	for (i = 0; i < len; i++) {
		if (h->cmdidx == cmdsize)
			h->databuf[datalen++] = *(data++);
		else
			h->cmdbuf[h->cmdidx++] = *(data++);
	}
	if (h->version.stlink == 1)
		stlink_usb_set_cbw_transfer_datalength(handle, datalen);

	res = stlink_cmd_allow_retry(handle, h->databuf, datalen);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

static int stlink_swim_readbytes(void *handle, uint32_t addr, uint32_t len, uint8_t *data)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	if (len > STLINK_DATA_SIZE)
		return ERROR_FAIL;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READMEM;
	h_u16_to_be(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h_u32_to_be(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;

	stlink_usb_init_buffer(handle, h->rx_ep, len);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READBUF;
	res = stlink_usb_xfer(handle, data, len);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_idcode(void *handle, uint32_t *idcode)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* there is no swim read core id cmd */
	if (h->transport == HL_TRANSPORT_SWIM) {
		*idcode = 0;
		return ERROR_OK;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 4);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READCOREID;

	res = stlink_usb_xfer(handle, h->databuf, 4);

	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->databuf);

	LOG_DEBUG("IDCODE: 0x%08" PRIX32, *idcode);

	return ERROR_OK;
}

static int stlink_usb_v2_read_debug_reg(void *handle, uint32_t addr, uint32_t *val)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 8);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READDEBUGREG;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h->cmdbuf[h->cmdidx++] = HLA_DEFAULT_APNUN;

	res = stlink_cmd_allow_retry(handle, h->databuf, 8);
	if (res != ERROR_OK)
		return res;

	*val = le_to_h_u32(h->databuf + 4);
	return ERROR_OK;
}

static int stlink_usb_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_WRITEDEBUGREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITEDEBUGREG;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u32_to_le(h->cmdbuf+h->cmdidx, val);
	h->cmdidx += 4;
	h->cmdbuf[h->cmdidx++] = HLA_DEFAULT_APNUN;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_trace_read(void *handle, uint8_t *buf, size_t *size)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->trace.enabled &&
		(h->version.jtag >= STLINK_TRACE_MIN_VERSION || h->jtag_api == STLINK_JTAG_API_V3)) {
		int res;

		stlink_usb_init_buffer(handle, h->rx_ep, 10);

		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_GET_TRACE_NB;

		res = stlink_usb_xfer(handle, h->databuf, 2);
		if (res != ERROR_OK)
			return res;

		size_t bytes_avail = le_to_h_u16(h->databuf);
		*size = bytes_avail < *size ? bytes_avail : *size - 1;

		if (*size > 0) {
			res = stlink_usb_read_trace(handle, buf, *size);
			if (res != ERROR_OK)
				return res;
			return ERROR_OK;
		}
	}
	*size = 0;
	return ERROR_OK;
}

static enum target_state stlink_usb_v2_get_status(void *handle)
{
	int result;
	uint32_t status;

	result = stlink_usb_v2_read_debug_reg(handle, DCB_DHCSR, &status);
	if  (result != ERROR_OK)
		return TARGET_UNKNOWN;

	if (status & S_HALT)
		return TARGET_HALTED;
	else if (status & S_RESET_ST)
		return TARGET_RESET;

	return TARGET_RUNNING;
}

/** */
static enum target_state stlink_usb_state(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->transport == HL_TRANSPORT_SWIM) {
		res = stlink_usb_mode_enter(handle, stlink_get_mode(h->transport));
		if (res != ERROR_OK)
			return TARGET_UNKNOWN;

		res = stlink_swim_resync(handle);
		if (res != ERROR_OK)
			return TARGET_UNKNOWN;

		return ERROR_OK;
	}

	if (h->reconnect_pending) {
		LOG_INFO("Previous state query failed, trying to reconnect");
		res = stlink_usb_mode_enter(handle, stlink_get_mode(h->transport));

		if (res != ERROR_OK)
			return TARGET_UNKNOWN;

		h->reconnect_pending = false;
	}

	if (h->jtag_api == STLINK_JTAG_API_V2 || h->jtag_api == STLINK_JTAG_API_V3) {
		res = stlink_usb_v2_get_status(handle);
		if (res == TARGET_UNKNOWN)
			h->reconnect_pending = true;
		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_GETSTATUS;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	if (h->databuf[0] == STLINK_CORE_RUNNING)
		return TARGET_RUNNING;
	if (h->databuf[0] == STLINK_CORE_HALTED)
		return TARGET_HALTED;

	h->reconnect_pending = true;

	return TARGET_UNKNOWN;
}

static int stlink_usb_assert_srst(void *handle, int srst)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->transport == HL_TRANSPORT_SWIM)
		return stlink_swim_assert_reset(handle, srst);

	if (h->version.stlink == 1)
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_DRIVE_NRST;
	h->cmdbuf[h->cmdidx++] = srst;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static void stlink_usb_trace_disable(void *handle)
{
	int res = ERROR_OK;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	assert(h->version.jtag >= STLINK_TRACE_MIN_VERSION || h->jtag_api == STLINK_JTAG_API_V3);

	LOG_DEBUG("Tracing: disable");

	stlink_usb_init_buffer(handle, h->rx_ep, 2);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_STOP_TRACE_RX;
	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res == ERROR_OK)
		h->trace.enabled = false;
}


/** */
static int stlink_usb_trace_enable(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.jtag >= STLINK_TRACE_MIN_VERSION || h->jtag_api == STLINK_JTAG_API_V3) {
		stlink_usb_init_buffer(handle, h->rx_ep, 10);

		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_START_TRACE_RX;
		h_u16_to_le(h->cmdbuf+h->cmdidx, (uint16_t)STLINK_TRACE_SIZE);
		h->cmdidx += 2;
		h_u32_to_le(h->cmdbuf+h->cmdidx, h->trace.source_hz);
		h->cmdidx += 4;

		res = stlink_usb_xfer(handle, h->databuf, 2);

		if (res == ERROR_OK)  {
			h->trace.enabled = true;
			LOG_DEBUG("Tracing: recording at %" PRIu32 "Hz", h->trace.source_hz);
		}
	} else {
		LOG_ERROR("Tracing is not supported by this version.");
		res = ERROR_FAIL;
	}

	return res;
}

/** */
static int stlink_usb_reset(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int retval;

	assert(handle != NULL);

	if (h->transport == HL_TRANSPORT_SWIM)
		return stlink_swim_generate_rst(handle);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;

	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_RESETSYS;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_RESETSYS;

	retval = stlink_cmd_allow_retry(handle, h->databuf, 2);
	if (retval != ERROR_OK)
		return retval;

	if (h->trace.enabled) {
		stlink_usb_trace_disable(h);
		return stlink_usb_trace_enable(h);
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_run(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V2 || h->jtag_api == STLINK_JTAG_API_V3) {
		res = stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_DEBUGEN);

		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_RUNCORE;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_halt(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V2 || h->jtag_api == STLINK_JTAG_API_V3) {
		res = stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);

		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_FORCEDEBUG;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_step(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V2 || h->jtag_api == STLINK_JTAG_API_V3) {
		/* TODO: this emulates the v1 api, it should really use a similar auto mask isr
		 * that the Cortex-M3 currently does. */
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_MASKINTS|C_DEBUGEN);
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_STEP|C_MASKINTS|C_DEBUGEN);
		return stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_STEPCORE;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_read_regs(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 84);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_READALLREGS;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READALLREGS;

	if (h->jtag_api >= STLINK_JTAG_API_V2 || h->jtag_api == STLINK_JTAG_API_V3)
		h->cmdbuf[h->cmdidx++] = HLA_DEFAULT_APNUN;

	res = stlink_usb_xfer(handle, h->databuf, 84);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_read_reg(void *handle, int num, uint32_t *val)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, h->jtag_api == STLINK_JTAG_API_V1 ? 4 : 8);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_READREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READREG;
	h->cmdbuf[h->cmdidx++] = num;

	if ((h->jtag_api == STLINK_JTAG_API_V2 && h->version.jtag >= 28) || h->jtag_api == STLINK_JTAG_API_V3)
		h->cmdbuf[h->cmdidx++] = HLA_DEFAULT_APNUN;

	if (h->jtag_api == STLINK_JTAG_API_V1) {
		res = stlink_usb_xfer(handle, h->databuf, 4);
		if (res != ERROR_OK)
			return res;
		*val = le_to_h_u32(h->databuf);
		return ERROR_OK;
	} else {
		res = stlink_cmd_allow_retry(handle, h->databuf, 8);
		if (res != ERROR_OK)
			return res;
		*val = le_to_h_u32(h->databuf + 4);
		return ERROR_OK;
	}
}

/** */
static int stlink_usb_write_reg(void *handle, int num, uint32_t val)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_WRITEREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITEREG;
	h->cmdbuf[h->cmdidx++] = num;
	h_u32_to_le(h->cmdbuf+h->cmdidx, val);
	h->cmdidx += 4;

	if ((h->jtag_api == STLINK_JTAG_API_V2 && h->version.jtag >= 28) || h->jtag_api == STLINK_JTAG_API_V3)
		h->cmdbuf[h->cmdidx++] = HLA_DEFAULT_APNUN;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

static int stlink_usb_get_rw_status(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V1)
		return ERROR_OK;

	stlink_usb_init_buffer(handle, h->rx_ep, 3);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	/* FIXME: this is incorrect */
	if (h->jtag_api == STLINK_JTAG_API_V3) {
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV3_GETLASTRWSTATUS;
		h->cmdbuf[h->cmdidx++] = HLA_DEFAULT_APNUN;
		res = stlink_usb_xfer(handle, h->databuf, 12);
	} else {
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_GETLASTRWSTATUS;
		if (h->version.jtag >= 28)
			h->cmdbuf[h->cmdidx++] = HLA_DEFAULT_APNUN;
		res = stlink_usb_xfer(handle, h->databuf, 2);
	}

	if (res != ERROR_OK)
		return res;

	return stlink_usb_error_check(h);
}

/** */
static int stlink_usb_read_mem8(void *handle, uint8_t ap_num, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	uint16_t read_len = len;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* max 8 bit read/write is 64 bytes or 512 bytes for v3 */
	if (len > stlink_usb_block(h)) {
		LOG_DEBUG("max buffer (%d) length exceeded", stlink_usb_block(h));
		return ERROR_FAIL;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, read_len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READMEM_8BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h->cmdbuf[h->cmdidx++] = ap_num;

	/* we need to fix read length for single bytes */
	if (read_len == 1)
		read_len++;

	res = stlink_usb_xfer(handle, h->databuf, read_len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem8(void *handle, uint8_t ap_num, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* max 8 bit read/write is 64 bytes or 512 bytes for v3 */
	if (len > stlink_usb_block(h)) {
		LOG_DEBUG("max buffer length (%d) exceeded", stlink_usb_block(h));
		return ERROR_FAIL;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_WRITEMEM_8BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h->cmdbuf[h->cmdidx++] = ap_num;

	res = stlink_usb_xfer(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_read_mem32(void *handle, uint8_t ap_num, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READMEM_32BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h->cmdbuf[h->cmdidx++] = ap_num;

	res = stlink_usb_xfer(handle, h->databuf, len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem32(void *handle, uint8_t ap_num, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_WRITEMEM_32BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h->cmdbuf[h->cmdidx++] = ap_num;

	res = stlink_usb_xfer(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_read_mem32_noaddrinc(void *handle, uint8_t ap_num, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READMEM_32BIT_NO_ADDR_INC;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h->cmdbuf[h->cmdidx++] = ap_num;

	res = stlink_usb_xfer(handle, h->databuf, len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem32_noaddrinc(void *handle, uint8_t ap_num, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_WRITEMEM_32BIT_NO_ADDR_INC;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h->cmdbuf[h->cmdidx++] = ap_num;

	res = stlink_usb_xfer(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

static uint32_t stlink_max_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	uint32_t max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));
	if (max_tar_block == 0)
		max_tar_block = 4;
	return max_tar_block;
}

static int stlink_usb_read_ap_mem(void *handle, uint8_t ap_num, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	int retries = 0;
	struct stlink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {

		bytes_remaining = (size == 4) ? \
				stlink_max_block_size(h->max_mem_packet, addr) : stlink_usb_block(h);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (h->transport == HL_TRANSPORT_SWIM) {
			retval = stlink_swim_readbytes(handle, addr, bytes_remaining, buffer);
			if (retval != ERROR_OK)
				return retval;
		} else
		/* the stlink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the stlink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the stlink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {

				uint32_t head_bytes = 4 - (addr % 4);
				retval = stlink_usb_read_mem8(handle, ap_num, addr, head_bytes, buffer);
				if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
					usleep((1<<retries++) * 1000);
					continue;
				}
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = stlink_usb_read_ap_mem(handle, ap_num, addr, 1, bytes_remaining, buffer);
			else
				retval = stlink_usb_read_mem32(handle, ap_num, addr, bytes_remaining, buffer);
		} else
			retval = stlink_usb_read_mem8(handle, ap_num, addr, bytes_remaining, buffer);

		if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			usleep((1<<retries++) * 1000);
			continue;
		}
		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int stlink_usb_read_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	return stlink_usb_read_ap_mem(handle, HLA_DEFAULT_APNUN, addr, size, count, buffer);
}

static int stlink_usb_write_ap_mem(void *handle, uint8_t ap_num, uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	int retries = 0;
	struct stlink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {

		bytes_remaining = (size == 4) ? \
				stlink_max_block_size(h->max_mem_packet, addr) : stlink_usb_block(h);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (h->transport == HL_TRANSPORT_SWIM) {
			retval = stlink_swim_writebytes(handle, addr, bytes_remaining, buffer);
			if (retval != ERROR_OK)
				return retval;
		} else
		/* the stlink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the stlink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the stlink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {

				uint32_t head_bytes = 4 - (addr % 4);
				retval = stlink_usb_write_mem8(handle, ap_num, addr, head_bytes, buffer);
				if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
					usleep((1<<retries++) * 1000);
					continue;
				}
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = stlink_usb_write_ap_mem(handle, ap_num, addr, 1, bytes_remaining, buffer);
			else
				retval = stlink_usb_write_mem32(handle, ap_num, addr, bytes_remaining, buffer);

		} else
			retval = stlink_usb_write_mem8(handle, ap_num, addr, bytes_remaining, buffer);
		if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			usleep((1<<retries++) * 1000);
			continue;
		}
		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int stlink_usb_write_mem(void *handle, uint32_t addr, uint32_t size,
        uint32_t count, const uint8_t *buffer)
{
	return stlink_usb_write_ap_mem(handle, HLA_DEFAULT_APNUN, addr, size, count, buffer);
}

/** */
static int stlink_usb_override_target(const char *targetname)
{
	return !strcmp(targetname, "cortex_m");
}

static int stlink_speed_swim(void *handle, int khz, bool query)
{
	/*
			we dont care what the khz rate is
			we only have low and high speed...
			before changing speed the SWIM_CSR HS bit
			must be updated
	 */
	if (khz == 0)
		stlink_swim_speed(handle, 0);
	else
		stlink_swim_speed(handle, 1);
	return khz;
}

static int stlink_match_speed_map(const struct speed_map *map, unsigned int map_size, int khz, bool query)
{
	unsigned int i;
	int speed_index = -1;
	int speed_diff = INT_MAX;
	int last_valid_speed = -1;
	bool match = true;

	for (i = 0; i < map_size; i++) {
		if (!map[i].speed)
			continue;
		last_valid_speed = i;
		if (khz == map[i].speed) {
			speed_index = i;
			break;
		} else {
			int current_diff = khz - map[i].speed;
			/* get abs value for comparison */
			current_diff = (current_diff > 0) ? current_diff : -current_diff;
			if ((current_diff < speed_diff) && khz >= map[i].speed) {
				speed_diff = current_diff;
				speed_index = i;
			}
		}
	}

	if (speed_index == -1) {
		/* this will only be here if we cannot match the slow speed.
		 * use the slowest speed we support.*/
		speed_index = last_valid_speed;
		match = false;
	} else if (i == map_size)
		match = false;

	if (!match && query) {
		LOG_INFO("Unable to match requested speed %d kHz, using %d kHz", \
				khz, stlink_khz_to_speed_map[speed_index].speed);
	}

	return speed_index;
}

static int stlink_speed_swd(void *handle, int khz, bool query)
{
	int speed_index;
	struct stlink_usb_handle_s *h = handle;

	/* only supported by stlink/v2 and for firmware >= 22 */
	if (h->version.stlink == 1 || h->version.jtag < 22)
		return khz;

	speed_index = stlink_match_speed_map(stlink_khz_to_speed_map,
		ARRAY_SIZE(stlink_khz_to_speed_map), khz, query);

	if (!query) {
		int result = stlink_usb_set_swdclk(h, stlink_khz_to_speed_map[speed_index].speed_divisor);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to set adapter speed");
			return khz;
		}
	}

	return stlink_khz_to_speed_map[speed_index].speed;
}

static int stlink_speed_jtag(void *handle, int khz, bool query)
{
	int speed_index;
	struct stlink_usb_handle_s *h = handle;

	/* only supported by stlink/v2 and for firmware >= 24 */
	if (h->version.stlink == 1 || h->version.jtag < 24)
		return khz;

	speed_index = stlink_match_speed_map(stlink_khz_to_speed_map_jtag,
		ARRAY_SIZE(stlink_khz_to_speed_map_jtag), khz, query);

	if (!query) {
		int result = stlink_usb_set_jtagclk(h, stlink_khz_to_speed_map_jtag[speed_index].speed_divisor);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to set adapter speed");
			return khz;
		}
	}

	return stlink_khz_to_speed_map_jtag[speed_index].speed;
}

void stlink_dump_speed_map(const struct speed_map *map, unsigned int map_size)
{
	unsigned int i;

	LOG_DEBUG("Supported clock speeds are:");
	for (i = 0; i < map_size; i++)
		if (map[i].speed)
			LOG_DEBUG("%d kHz", map[i].speed);
}

static int stlink_get_com_freq(void *handle, bool is_jtag, struct speed_map *map)
{
	struct stlink_usb_handle_s *h = handle;
	int i;

	if (h->version.stlink < 3) {
		LOG_ERROR("Unknown command");
		return 0;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 16);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_APIV3_GET_COM_FREQ;
	h->cmdbuf[h->cmdidx++] = is_jtag ? 1 : 0;

	int res = stlink_usb_xfer(handle, h->databuf, 52);

	int size = h->databuf[8];

	if (size > STLINK_V3_MAX_FREQ_NB)
		size = STLINK_V3_MAX_FREQ_NB;

	for (i = 0; i < size; i++) {
		map[i].speed = le_to_h_u32(&h->databuf[12 + 4 * i]);
		map[i].speed_divisor = i;
	}

	/* set to zero all the next entries */
	for (i = size; i < STLINK_V3_MAX_FREQ_NB; i++)
		map[i].speed = 0;

	return res;
}

static int stlink_set_com_freq(void *handle, bool is_jtag, unsigned int frequency)
{
	struct stlink_usb_handle_s *h = handle;

	if (h->version.stlink < 3) {
		LOG_ERROR("Unknown command");
		return 0;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 16);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_APIV3_SET_COM_FREQ;
	h->cmdbuf[h->cmdidx++] = is_jtag ? 1 : 0;
	h->cmdbuf[h->cmdidx++] = 0;

	h_u32_to_le(&h->cmdbuf[4], frequency);

	return stlink_usb_xfer(handle, h->databuf, 8);
}

static int stlink_speed_v3(void *handle, int khz, bool query)
{
	struct stlink_usb_handle_s *h = handle;
	int speed_index;
	struct speed_map map[STLINK_V3_MAX_FREQ_NB];

	stlink_get_com_freq(h, (h->transport == HL_TRANSPORT_JTAG), map);

	speed_index = stlink_match_speed_map(map, ARRAY_SIZE(map), khz, query);

	if (!query) {
		int result = stlink_set_com_freq(h, (h->transport == HL_TRANSPORT_JTAG), map[speed_index].speed);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to set adapter speed");
			return khz;
		}
	}
	return map[speed_index].speed;
}

static int stlink_speed(void *handle, int khz, bool query)
{
	struct stlink_usb_handle_s *h = handle;

	if (!handle)
		return khz;

	if (h->transport == HL_TRANSPORT_SWIM)
		return stlink_speed_swim(handle, khz, query);
	else if (h->version.stlink == 3)
		return stlink_speed_v3(handle, khz, query);
	else if (h->transport == HL_TRANSPORT_SWD)
		return stlink_speed_swd(handle, khz, query);
	else if (h->transport == HL_TRANSPORT_JTAG)
		return stlink_speed_jtag(handle, khz, query);

	return khz;
}

/** */
static int stlink_usb_close(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	if (h && h->fd) {
		stlink_exit_mode(h);
		jtag_libusb_close(h->fd);
	}

	free(h);

	return ERROR_OK;
}

/** */
static int stlink_usb_open(struct hl_interface_param_s *param, void **fd)
{
	int err, retry_count = 1;
	struct stlink_usb_handle_s *h;
	enum stlink_jtag_api_version api;

	LOG_DEBUG("stlink_usb_open");

	h = calloc(1, sizeof(struct stlink_usb_handle_s));

	if (h == 0) {
		LOG_DEBUG("malloc failed");
		return ERROR_FAIL;
	}

	h->transport = param->transport;

	for (unsigned i = 0; param->vid[i]; i++) {
		LOG_DEBUG("transport: %d vid: 0x%04x pid: 0x%04x serial: %s",
			  param->transport, param->vid[i], param->pid[i],
			  param->serial ? param->serial : "");
	}

	/*
	  On certain host USB configurations(e.g. MacBook Air)
	  STLINKv2 dongle seems to have its FW in a funky state if,
	  after plugging it in, you try to use openocd with it more
	  then once (by launching and closing openocd). In cases like
	  that initial attempt to read the FW info via
	  stlink_usb_version will fail and the device has to be reset
	  in order to become operational.
	 */
	do {
		if (jtag_libusb_open(param->vid, param->pid, param->serial, &h->fd) != ERROR_OK) {
			LOG_ERROR("open failed");
			goto error_open;
		}

		jtag_libusb_set_configuration(h->fd, 0);

		if (jtag_libusb_claim_interface(h->fd, 0) != ERROR_OK) {
			LOG_DEBUG("claim interface failed");
			goto error_open;
		}

		/* RX EP is common for all versions */
		h->rx_ep = STLINK_RX_EP;

		uint16_t pid;
		if (jtag_libusb_get_pid(jtag_libusb_get_device(h->fd), &pid) != ERROR_OK) {
			LOG_DEBUG("libusb_get_pid failed");
			goto error_open;
		}

		/* wrap version for first read */
		switch (pid) {
			case STLINK_V1_PID:
				h->version.stlink = 1;
				h->tx_ep = STLINK_TX_EP;
				h->trace_ep = STLINK_TRACE_EP;
				break;
			case STLINK_V3_DFU_PID:
			case STLINK_V3E_PID:
			case STLINK_V3S_PID:
			case STLINK_V3_2VCP_PID:
				h->version.stlink = 3;
				h->tx_ep = STLINK_V2_1_TX_EP;
				h->trace_ep = STLINK_V2_1_TRACE_EP;
				break;
			case STLINK_V2_1_PID:
			case STLINK_V2_1_NO_MSD_PID:
				h->version.stlink = 2;
				h->tx_ep = STLINK_V2_1_TX_EP;
				h->trace_ep = STLINK_V2_1_TRACE_EP;
				break;
			default:
			/* fall through - we assume V2 to be the default version*/
			case STLINK_V2_PID:
				h->version.stlink = 2;
				h->tx_ep = STLINK_TX_EP;
				h->trace_ep = STLINK_TRACE_EP;
				break;
		}
		h->pid = pid;

		/* get the device version */
		err = stlink_usb_version(h);

		if (err == ERROR_OK) {
			break;
		} else if (h->version.stlink == 1 ||
			   retry_count == 0) {
			LOG_ERROR("read version failed");
			goto error_open;
		} else {
			err = jtag_libusb_release_interface(h->fd, 0);
			if (err != ERROR_OK) {
				LOG_ERROR("release interface failed");
				goto error_open;
			}

			err = jtag_libusb_reset_device(h->fd);
			if (err != ERROR_OK) {
				LOG_ERROR("reset device failed");
				goto error_open;
			}

			jtag_libusb_close(h->fd);
			/*
			  Give the device one second to settle down and
			  reenumerate.
			 */
			usleep(1 * 1000 * 1000);
			retry_count--;
		}
	} while (1);

	/* check if mode is supported */
	err = ERROR_OK;

	switch (h->transport) {
		case HL_TRANSPORT_SWD:
		case HL_TRANSPORT_JTAG:
			if (h->version.jtag == 0)
				err = ERROR_FAIL;
			break;
		case HL_TRANSPORT_SWIM:
			if (h->version.swim == 0)
				err = ERROR_FAIL;
			break;
		default:
			err = ERROR_FAIL;
			break;
	}

	if (err != ERROR_OK) {
		LOG_ERROR("mode (transport) not supported by device");
		goto error_open;
	}

	api = h->version.jtag_api_max;

	LOG_INFO("using stlink api v%d", api);

	/* set the used jtag api, this will default to the newest supported version */
	h->jtag_api = api;

	/* initialize the debug hardware */
	err = stlink_usb_init_mode(h, param->connect_under_reset);

	if (err != ERROR_OK) {
		LOG_ERROR("init mode failed (unable to connect to the target)");
		goto error_open;
	}

	if (h->transport == HL_TRANSPORT_SWIM) {
		err = stlink_swim_enter(h);
		if (err != ERROR_OK) {
			LOG_ERROR("stlink_swim_enter_failed (unable to connect to the target)");
			goto error_open;
		}
		*fd = h;
		h->max_mem_packet = STLINK_DATA_SIZE;
		return ERROR_OK;
	}

	if (h->transport == HL_TRANSPORT_JTAG) {
		/* jtag clock speed only supported by stlink/v2 and for firmware >= 24 */
		if (h->version.stlink == 2 && h->version.jtag >= 24) {
			stlink_dump_speed_map(stlink_khz_to_speed_map_jtag, ARRAY_SIZE(stlink_khz_to_speed_map_jtag));
			stlink_speed(h, param->initial_interface_speed, false);
		}
	} else if (h->transport == HL_TRANSPORT_SWD) {
		/* clock speed only supported by stlink/v2 and for firmware >= 22 */
		if (h->version.stlink == 2 && h->version.jtag >= 22) {
			stlink_dump_speed_map(stlink_khz_to_speed_map, ARRAY_SIZE(stlink_khz_to_speed_map));
			stlink_speed(h, param->initial_interface_speed, false);
		}
	}

	if (h->version.stlink == 3) {
		struct speed_map map[STLINK_V3_MAX_FREQ_NB];

		stlink_get_com_freq(h, (h->transport == HL_TRANSPORT_JTAG), map);
		stlink_dump_speed_map(map, ARRAY_SIZE(map));
		stlink_speed(h, param->initial_interface_speed, false);
	}

	/* get cpuid, so we can determine the max page size
	 * start with a safe default */
	h->max_mem_packet = (1 << 10);

#if 0
	uint8_t buffer[4];
	err = stlink_usb_read_mem32(h, 0, CPUID, 4, buffer);
	if (err == ERROR_OK) {
		uint32_t cpuid = le_to_h_u32(buffer);
		int i = (cpuid >> 4) & 0xf;
		if (i == 4 || i == 3) {
			/* Cortex-M3/M4 has 4096 bytes autoincrement range */
			h->max_mem_packet = (1 << 12);
		}
	}
#endif

	LOG_DEBUG("Using TAR autoincrement: %" PRIu32, h->max_mem_packet);

	*fd = h;

	return ERROR_OK;

error_open:
	stlink_usb_close(h);

	return ERROR_FAIL;
}

int stlink_config_trace(void *handle, bool enabled, enum tpiu_pin_protocol pin_protocol,
			uint32_t port_size, unsigned int *trace_freq)
{
	struct stlink_usb_handle_s *h = handle;

	if (enabled && (h->jtag_api < 2 ||
			pin_protocol != TPIU_PIN_PROTOCOL_ASYNC_UART)) {
		LOG_ERROR("The attached ST-LINK version doesn't support this trace mode");
		return ERROR_FAIL;
	}

	if (!enabled) {
		stlink_usb_trace_disable(h);
		return ERROR_OK;
	}

	if (*trace_freq > STLINK_TRACE_MAX_HZ) {
		LOG_ERROR("ST-LINK doesn't support SWO frequency higher than %u",
			  STLINK_TRACE_MAX_HZ);
		return ERROR_FAIL;
	}

	stlink_usb_trace_disable(h);

	if (!*trace_freq)
		*trace_freq = STLINK_TRACE_MAX_HZ;
	h->trace.source_hz = *trace_freq;

	return stlink_usb_trace_enable(h);
}

/** */
static int stlink_usb_init_access_point(void *handle,
		unsigned char access_point_id, unsigned char resource)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* only required by stlink/v2 and for firmware >= 28 */
	if (h->version.stlink == 1 || (h->version.stlink == 2 && h->version.jtag < 28))
		return ERROR_OK;

	LOG_DEBUG_IO("init apnum = %d, resource %d", access_point_id, resource);
	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_INIT_AP;
	h->cmdbuf[h->cmdidx++] = access_point_id;
	h->cmdbuf[h->cmdidx++] = resource;
	h_u32_to_le(&h->cmdbuf[12], 0);

	return stlink_usb_xfer(handle, h->databuf, 2);
}

/** */
static int stlink_usb_close_access_point(void *handle,
		unsigned char access_point_id)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* only required by stlink/v2 and for firmware >= 28 */
	if (h->version.stlink == 1 || (h->version.stlink == 2 && h->version.jtag < 28))
		return ERROR_OK;

	LOG_DEBUG_IO("close apnum = %d", access_point_id);
	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_CLOSE_AP_DBG;
	h->cmdbuf[h->cmdidx++] = access_point_id;
	h_u32_to_le(&h->cmdbuf[12], 0);

	return stlink_usb_xfer(handle, h->databuf, 2);
}

/** */
static int stlink_internal_read_dap_register(void *handle, unsigned short dap_port,
			unsigned short addr, uint32_t *val)
{
	struct stlink_usb_handle_s *h = handle;
	int retval;

	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_JTAG_READ_DAP_REG;
	h_u16_to_le(&h->cmdbuf[2], dap_port);
	h_u16_to_le(&h->cmdbuf[4], addr);
	h_u32_to_le(&h->cmdbuf[12], 0);

	retval = stlink_usb_xfer(handle, h->databuf, 8);
	*val = le_to_h_u32(h->databuf + 4);
	return retval;
}

#define STLINK_DEBUG_PORT 0xffff
static int stlink_read_dap_register(void *handle, unsigned short dap_port,
			unsigned short addr, uint32_t *val)
{
	struct stlink_usb_handle_s *h = handle;
	int retval;
	uint32_t dummy;

	assert(handle != NULL);

	/* only supported by stlink/v2 and for firmware >= 24 */
	if (h->version.stlink == 1 || (h->version.stlink == 2 && h->version.jtag < 24))
		return ERROR_COMMAND_NOTFOUND;

	if (h->transport == HL_TRANSPORT_JTAG && dap_port == STLINK_DEBUG_PORT
		&& h->version.stlink == 2 && h->version.jtag < 32) {
		/* workaround for V2J24 ~ V2J31 */
		retval = stlink_internal_read_dap_register(handle, STLINK_DEBUG_PORT, addr, &dummy);
		if (retval != ERROR_OK)
			return retval;
		retval = stlink_internal_read_dap_register(handle, STLINK_DEBUG_PORT, DP_RDBUFF, val);
	} else {
		retval = stlink_internal_read_dap_register(handle, dap_port, addr, val);
	}

	LOG_DEBUG_IO("dap_port_read = %d, addr =  0x%x, value = 0x%x", dap_port, addr, *val);
	return retval;
}

/** */
static int stlink_write_dap_register(void *handle, unsigned short dap_port,
			unsigned short addr, uint32_t val)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* only supported by stlink/v2 and for firmware >= 24 */
	if (h->version.stlink == 1 || (h->version.stlink == 2 && h->version.jtag < 24))
		return ERROR_COMMAND_NOTFOUND;

	LOG_DEBUG_IO("dap_port_write = %d, addr = 0x%x, value = 0x%x", dap_port, addr, val);
	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_JTAG_WRITE_DAP_REG;
	h_u16_to_le(&h->cmdbuf[2], dap_port);
	h_u16_to_le(&h->cmdbuf[4], addr);
	h_u32_to_le(&h->cmdbuf[6], val);
	h_u32_to_le(&h->cmdbuf[12], 0);
	return stlink_usb_xfer(handle, h->databuf, 2);
}

/** */
struct hl_layout_api_s stlink_usb_layout_api = {
	/** */
	.open = stlink_usb_open,
	/** */
	.close = stlink_usb_close,
	/** */
	.idcode = stlink_usb_idcode,
	/** */
	.state = stlink_usb_state,
	/** */
	.reset = stlink_usb_reset,
	/** */
	.assert_srst = stlink_usb_assert_srst,
	/** */
	.run = stlink_usb_run,
	/** */
	.halt = stlink_usb_halt,
	/** */
	.step = stlink_usb_step,
	/** */
	.read_regs = stlink_usb_read_regs,
	/** */
	.read_reg = stlink_usb_read_reg,
	/** */
	.write_reg = stlink_usb_write_reg,
	/** */
	.read_mem = stlink_usb_read_mem,
	/** */
	.write_mem = stlink_usb_write_mem,
	/** */
	.write_debug_reg = stlink_usb_write_debug_reg,
	/** */
	.override_target = stlink_usb_override_target,
	/** */
	.speed = stlink_speed,
	/** */
	.config_trace = stlink_config_trace,
	/** */
	.poll_trace = stlink_usb_trace_read,
};

/* Low-level interface */

static struct stlink_usb_handle_s *stlink_dap_handle;
static struct hl_interface_param_s stlink_dap_param = {
	.transport = HL_TRANSPORT_JTAG,
	.vid = {STLINK_VID,    STLINK_VID,    STLINK_VID,      STLINK_VID,             STLINK_VID,        STLINK_VID,     STLINK_VID,     STLINK_VID,         0},
	.pid = {STLINK_V1_PID, STLINK_V2_PID, STLINK_V2_1_PID, STLINK_V2_1_NO_MSD_PID, STLINK_V3_DFU_PID, STLINK_V3E_PID, STLINK_V3S_PID, STLINK_V3_2VCP_PID, 0},
};
static DECLARE_BITMAP(opened_ap, DP_APSEL_MAX + 1);
static uint8_t ap_csw_size_cached[DP_APSEL_MAX + 1];

static int stlink_dap_open_ap(unsigned short apsel)
{
	int retval;

	if (apsel > DP_APSEL_MAX)
		return ERROR_OK;

	if (test_bit(apsel, opened_ap))
		return ERROR_OK;

	retval = stlink_usb_init_access_point(stlink_dap_handle, apsel, 0);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("AP %d enabled", apsel);
	set_bit(apsel, opened_ap);
	return ERROR_OK;
}

static int stlink_dap_closeall_ap(void)
{
	int retval, apsel;

	for (apsel = 0; apsel <= DP_APSEL_MAX; apsel++) {
		if (!test_bit(apsel, opened_ap))
			continue;
		retval = stlink_usb_close_access_point(stlink_dap_handle, apsel);
		if (retval != ERROR_OK)
			return retval;
		clear_bit(apsel, opened_ap);
	}
	return ERROR_OK;
}

int stlink_dap_dap_read(unsigned short dap_port, unsigned short addr, uint32_t *val)
{
	uint32_t dummy;
	int retval;

	/* Skip opening DAP if we are only scanning to look for an AP */
	if (dap_port <= DP_APSEL_MAX && addr != AP_REG_IDR) {
		retval = stlink_dap_open_ap(dap_port);
		if (retval != ERROR_OK)
			return retval;
	}

	if (!val)
		val = &dummy;
	return stlink_read_dap_register(stlink_dap_handle, dap_port, addr, val);
}

int stlink_dap_dap_write(unsigned short dap_port, unsigned short addr, uint32_t val)
{
	int retval;

	retval = stlink_dap_open_ap(dap_port);
	if (retval != ERROR_OK)
		return retval;

	return stlink_write_dap_register(stlink_dap_handle, dap_port, addr, val);
}

/*
 * Workaround for setting CSW with ST-Link pre-versions V2J32 and V3J2.
 * Before ST-Link versions above, high level API does not provide a method to
 * set CSW. Same issue on every version of ST-Link V1. This forced us using
 * the un-efficient low-level AP register API for every memory read/write.
 *
 * This workaround leverage the CSW caching operated by ST-Link. At every
 * memory R/W, ST-Link computes the new CSW value based on word size. If it
 * match the previous CSW value than it has wrote in CSW register, ST-Link
 * will not write in CSW register again.
 *
 * Here we track the word size used in the last memory R/W. If it does not
 * match with current word size, we first force ST-Link to update CSW register
 * and its internal cache accordingly to the new size. Then we overwrite CSW
 * register with the value we need.
 * The following memory R/W will operate base on the new CSW and ST-Link will
 * not change it.
 */
static int stlink_dap_set_csw(struct adiv5_ap *ap, uint32_t size, bool addrinc)
{
	uint32_t csw;
	uint8_t dummy[4], ap_num;
	int retval;

	ap_num = ap->ap_num;

	struct stlink_usb_handle_s *h = stlink_dap_handle;
	if ((h->version.stlink == 2 && h->version.jtag >= 32) || (h->version.stlink == 3 && h->version.jtag >= 2)) {
		csw = ap->csw_default;
		if (csw != (ap->csw_value & ~(CSW_SIZE_MASK | CSW_ADDRINC_MASK))) {
			retval = dap_queue_ap_write(ap, MEM_AP_REG_CSW, csw);
			if (retval != ERROR_OK) {
				ap->csw_value = 0;
				return retval;
			}
			ap->csw_value = csw;
		}
		return ERROR_OK;
	}

	switch (size) {
	case 2:
		/* current implementation only use 8 and 32 bits */
		size = 1;
		/* fallthrough */
	case 1:
		csw = CSW_8BIT;
		break;
	case 4:
	default:
		/* ST-Link sets autoinc only in 32 bits mode */
		/*
		 * ARM IHI 0031D: Note:
		 * It is IMPLEMENTATION DEFINED whether a MEM-AP supports transfer sizes other than Word. If a
		 * MEM-AP only supports word transfers and Increment single is selected, the TAR always
		 * increments by four after a successful DRW transaction.
		 */
		csw = addrinc ? (CSW_32BIT | CSW_ADDRINC_SINGLE) : CSW_32BIT;
		break;
	}
	csw |= ap->csw_default;

	if (ap_csw_size_cached[ap_num] != size) {
		ap_csw_size_cached[ap_num] = size;

		/* The mem read below will change CSW */
		ap->csw_value = 0;
		stlink_usb_read_ap_mem(stlink_dap_handle, ap_num, 0x00000000, size, 1, dummy);
	}

	if (ap->csw_value != csw) {
		retval = dap_queue_ap_write(ap, MEM_AP_REG_CSW, csw);
		if (retval != ERROR_OK) {
			ap->csw_value = 0;
			return retval;
		}
		ap->csw_value = csw;
	}
	return ERROR_OK;
}

static int stlink_dap_reset_csw(struct adiv5_ap *ap)
{
	uint32_t csw;
	int retval;

	csw = ap->csw_value;
	if (csw & CSW_ADDRINC_SINGLE)
		return ERROR_OK;

	csw &= ~CSW_ADDRINC_SINGLE;
	retval = dap_queue_ap_write(ap, MEM_AP_REG_CSW, csw);
	if (retval != ERROR_OK)
		return retval;
	ap->csw_value = csw;
	return ERROR_OK;
}

int stlink_dap_ap_mem_read(struct adiv5_ap *ap, uint8_t *buffer,
	uint32_t size, uint32_t count, uint32_t address, bool addrinc)
{
	int retval;
	uint8_t ap_num;
	uint32_t partial;
	int retries = 0;
	uint32_t bytes_remaining;

	if (!addrinc && size != 4)
		return ERROR_OP_NOT_SUPPORTED;

	struct stlink_usb_handle_s *h = stlink_dap_handle;
	if (!addrinc && (h->version.stlink == 1 || (h->version.stlink == 2 && h->version.jtag < 24)))
		return ERROR_OP_NOT_SUPPORTED;

	ap_num = ap->ap_num;
	retval = stlink_dap_open_ap(ap_num);
	if (retval != ERROR_OK)
		return retval;

	if (size == 4 && address & 3) {
		/* unaligned 32-bits read could be split in mix 32 & 8-bits. Force 8-bis only */
		count *= 4;
		size = 1;
	}

	/* here we do not track TAR, and next calls will change it */
	ap->tar_valid = false;

	retval = stlink_dap_set_csw(ap, size, addrinc);
	if (retval != ERROR_OK)
		return retval;

	if (addrinc)
		return stlink_usb_read_ap_mem(stlink_dap_handle, ap_num, address, size,
			count, buffer);

	/* !addrinc && size == 4 */

	if ((h->version.stlink == 2 && h->version.jtag >= 32) || (h->version.stlink == 3 && h->version.jtag >= 2)) {
		count *= 4;
		while (count) {
			bytes_remaining = stlink_usb_block(h);

			if (count < bytes_remaining)
				bytes_remaining = count;

			retval = stlink_usb_read_mem32_noaddrinc(stlink_dap_handle, ap_num, address, bytes_remaining, buffer);
			if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
				usleep((1<<retries++) * 1000);
				continue;
			}
			if (retval != ERROR_OK)
				return retval;

			retries = 0;
			buffer += bytes_remaining;
			count -= bytes_remaining;
		}
		return ERROR_OK;
	}

	/* workaround for V2J24..V2J31 and V3J1 */
	partial = (TAR_AUTOINCR_BLOCK - (address & (TAR_AUTOINCR_BLOCK - 1))) / 4;
	if (partial > stlink_usb_block(stlink_dap_handle) / 4)
		partial = stlink_usb_block(stlink_dap_handle) / 4;
	while (count) {
		if (partial > count)
			partial = count;
		retval = stlink_usb_read_mem32(stlink_dap_handle, ap_num, address, 4 * partial, buffer);
		if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			usleep((1<<retries++) * 1000);
			continue;
		}
		if (retval != ERROR_OK)
			return retval;
		retries = 0;
		count -= partial;
		buffer += 4 * partial;
	}
	return stlink_dap_reset_csw(ap);
}

int stlink_dap_ap_mem_write(struct adiv5_ap *ap, const uint8_t *buffer,
	uint32_t size, uint32_t count, uint32_t address, bool addrinc)
{
	int retval;
	uint8_t ap_num;
	uint32_t partial;
	int retries = 0;
	uint32_t bytes_remaining;

	if (!addrinc && size != 4)
		return ERROR_OP_NOT_SUPPORTED;

	struct stlink_usb_handle_s *h = stlink_dap_handle;
	if (!addrinc && (h->version.stlink == 1 || (h->version.stlink == 2 && h->version.jtag < 24)))
		return ERROR_OP_NOT_SUPPORTED;

	ap_num = ap->ap_num;
	retval = stlink_dap_open_ap(ap_num);
	if (retval != ERROR_OK)
		return retval;

	if (size == 4 && address & 3) {
		/* unaligned 32-bits write could be split in mix 32 & 8-bits. Force 8-bis only */
		count *= 4;
		size = 1;
	}

	/* here we do not track TAR, and next calls will change it */
	ap->tar_valid = false;

	retval = stlink_dap_set_csw(ap, size, addrinc);
	if (retval != ERROR_OK)
		return retval;

	if (addrinc)
		return stlink_usb_write_ap_mem(stlink_dap_handle, ap_num, address, size,
			count, buffer);

	/* !addrinc && size == 4 */

	if ((h->version.stlink == 2 && h->version.jtag >= 32) || (h->version.stlink == 3 && h->version.jtag >= 2)) {
		count *= 4;
		while (count) {
			bytes_remaining = stlink_usb_block(h);

			if (count < bytes_remaining)
				bytes_remaining = count;

			retval = stlink_usb_write_mem32_noaddrinc(stlink_dap_handle, ap_num, address, bytes_remaining, buffer);
			if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
				usleep((1<<retries++) * 1000);
				continue;
			}
			if (retval != ERROR_OK)
				return retval;

			retries = 0;
			buffer += bytes_remaining;
			count -= bytes_remaining;
		}
		return ERROR_OK;
	}

	/* workaround for V2J24..V2J31 and V3J1 */
	partial = (TAR_AUTOINCR_BLOCK - (address & (TAR_AUTOINCR_BLOCK - 1))) / 4;
	if (partial > stlink_usb_block(stlink_dap_handle) / 4)
		partial = stlink_usb_block(stlink_dap_handle) / 4;
	while (count) {
		if (partial > count)
			partial = count;
		retval = stlink_usb_write_mem32(stlink_dap_handle, ap_num, address, 4 * partial, buffer);
		if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			usleep((1<<retries++) * 1000);
			continue;
		}
		if (retval != ERROR_OK)
			return retval;
		retries = 0;
		count -= partial;
		buffer += 4 * partial;
	}
	return stlink_dap_reset_csw(ap);
}

static int stlink_dap_swd_init(void)
{
	LOG_DEBUG("stlink_dap_swd_init()");
	stlink_dap_param.transport = HL_TRANSPORT_SWD;
	return ERROR_OK;
}

static int stlink_dap_speed(int speed)
{
	if (speed == 0) {
		LOG_ERROR("RTCK not supported. Set nonzero adapter_khz.");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	stlink_dap_param.initial_interface_speed = speed;
	stlink_speed(stlink_dap_handle, speed, false);
	return ERROR_OK;
}

static int stlink_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int stlink_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

static int_least32_t stlink_dap_swd_frequency(int_least32_t hz)
{
	if (hz > 0)
		stlink_dap_speed(hz / 1000);

	return hz;
}

static int stlink_dap_swd_switch_seq(enum swd_special_seq seq)
{
	LOG_ERROR("stlink_dap_swd_switch_seq()");
	assert(0);
	return ERROR_FAIL;
}

static void stlink_dap_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	LOG_ERROR("stlink_dap_swd_read_reg()");
	assert(0);
}

static void stlink_dap_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	LOG_ERROR("stlink_dap_swd_write_reg()");
	assert(0);
}

static int stlink_dap_swd_run_queue(void)
{
	LOG_ERROR("stlink_dap_swd_run_queue()");
	assert(0);
	return ERROR_FAIL;
}

static void stlink_dap_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
	case JTAG_RESET:
		LOG_DEBUG("stlink_usb_assert trst(%d) srst(%d)", !cmd->cmd.reset->trst, !cmd->cmd.reset->srst);
		if (cmd->cmd.reset->trst)
			stlink_usb_reset(stlink_dap_handle);
		stlink_usb_assert_srst(stlink_dap_handle, cmd->cmd.reset->srst ? STLINK_DEBUG_APIV2_DRIVE_NRST_LOW : STLINK_DEBUG_APIV2_DRIVE_NRST_HIGH);
		break;
	case JTAG_SLEEP:
		jtag_sleep(cmd->cmd.sleep->us);
		break;
	default:
		LOG_ERROR("stlink_dap_execute_queue(%d)", cmd->type);
		assert(0);
		break;
	}
}

static int stlink_dap_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	LOG_DEBUG_IO("stlink_dap_execute_queue()");

	while (cmd != NULL) {
		stlink_dap_execute_command(cmd);
		cmd = cmd->next;
	}

	return ERROR_OK;
}

static int stlink_dap_init(void)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	LOG_DEBUG("stlink_dap_init()");

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			stlink_dap_param.connect_under_reset = true;
		else
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
	}
	return stlink_usb_open(&stlink_dap_param, (void **)&stlink_dap_handle);
}

static int stlink_dap_quit(void)
{
	int retval;

	LOG_DEBUG("stlink_dap_quit()");

	retval = stlink_dap_closeall_ap();
	if (retval != ERROR_OK)
		return retval;

	free((void *)stlink_dap_param.serial);
	stlink_dap_param.serial = NULL;

	return stlink_usb_close(stlink_dap_handle);
}

COMMAND_HANDLER(stlink_dap_serial_command)
{
	LOG_DEBUG("stlink_dap_serial_command");

	if (CMD_ARGC != 1) {
		LOG_ERROR("Expected exactly one argument for \"st-link serial <serial-number>\".");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (stlink_dap_param.serial) {
		LOG_WARNING("Command \"stlink serial\" already used. Replace previous value");
		free((void *)stlink_dap_param.serial);
	}

	stlink_dap_param.serial = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

COMMAND_HANDLER(stlink_dap_arp_init)
{
	if (stlink_dap_param.transport == HL_TRANSPORT_JTAG ||
		(stlink_dap_param.transport == HL_TRANSPORT_SWD && stlink_dap_handle->version.stlink == 3)) {
		//stlink_usb_reset(stlink_dap_handle);
		stlink_usb_mode_leave(stlink_dap_handle, STLINK_MODE_DEBUG_JTAG);
		stlink_usb_mode_enter(stlink_dap_handle, stlink_get_mode(stlink_dap_param.transport));
		/* exit jtag closes all the opened AP; reopen them! */
		for (int apsel = 0; apsel <= DP_APSEL_MAX; apsel++)
			if (test_bit(apsel, opened_ap)) {
				clear_bit(apsel, opened_ap);
				stlink_dap_open_ap(apsel);
			}
		/* equivalent to
			jtag_add_tlr();
			jtag_execute_queue();
		but let's stay away from jtag code */
	}
	return ERROR_OK;
}

static const struct command_registration stlink_dap_subcommand_handlers[] = {
	{
		.name = "serial",
		.handler = &stlink_dap_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the device that should be used",
		.usage = "<serial number>",
	},
	{
		.name = "arp_init",
		.handler = &stlink_dap_arp_init,
		.mode = COMMAND_ANY,
		.help = "Validates JTAG scan chain against the list of "
				"declared TAPs.",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stlink_dap_command_handlers[] = {
	{
		.name = "st-link",
		.mode = COMMAND_ANY,
		.help = "perform st-link management",
		.chain = stlink_dap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver stlink_dap_swd_driver = {
	.init = stlink_dap_swd_init,
	.frequency = stlink_dap_swd_frequency,
	.switch_seq = stlink_dap_swd_switch_seq,
	.read_reg = stlink_dap_swd_read_reg,
	.write_reg = stlink_dap_swd_write_reg,
	.run = stlink_dap_swd_run_queue,
};

static const char *const stlink_dap_transport[] = { "stlink_swd", "stlink_jtag", NULL };

struct jtag_interface stlink_dap_interface = {
	.name = "st-link",
	.commands = stlink_dap_command_handlers,
	.swd = &stlink_dap_swd_driver,
	.transports = stlink_dap_transport,

	.execute_queue = stlink_dap_execute_queue,
	.speed = stlink_dap_speed,
	.speed_div = stlink_dap_speed_div,
	.khz = stlink_dap_khz,
	.init = stlink_dap_init,
	.quit = stlink_dap_quit,
};

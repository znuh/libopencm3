/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2014 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include "../usb/usb_private.h"
#include "common/st_usbfs_core.h"

/** Initialize the USB device controller hardware of the STM32. */
static usbd_device *st_usbfs_v3_usbd_init(void)
{
	uint32_t t_startup = 32;

	/* make things easier for user by handling sane defaults */
	if(rcc_get_usbclk_source() == RCC_HSIUSB48) {
		rcc_osc_on(RCC_HSIUSB48);
		rcc_wait_for_osc_ready(RCC_HSIUSB48);
		crs_autotrim_usb_enable();
	}

	/* we need to keep reset enabled for t_STARTUP after 
	 * clearing powerdown or the transceiver won't work yet
	 * 
	 * experiments showed that not doing this will cause the
	 * USB interface to enter error mode (USB_ISTR_ERR set) */
	SET_REG(USB_CNTR_REG, USB_CNTR_FRES);

	/* datasheet states t_STARTUP: 1us
	 * we just wait 36 iterations with >= 2 cycles per iteration */
	do {
		__asm__("nop");
	} while(--t_startup);

	rcc_periph_clock_enable(RCC_USB);
	SET_REG(USB_CNTR_REG, 0);
	SET_REG(USB_ISTR_REG, 0);

	/* Enable RESET, SUSPEND, RESUME and CTR interrupts. */
	SET_REG(USB_CNTR_REG, USB_CNTR_RESETM | USB_CNTR_CTRM |
		USB_CNTR_SUSPM | USB_CNTR_WKUPM);
	SET_REG(USB_BCDR_REG, USB_BCDR_DPPU);
	return &st_usbfs_dev;
}

/**
 * Assign a data buffer in packet memory for an endpoint
 *
 * @param ep_id Endpoint ID (0..7)
 * @param dir_tx 1 if TX endpoint, 0 for RX
 * @param ram_ofs Pointer to RAM offset for packet buffer
 * @param rx_blocks BLSIZE / NUM_BLOCK[4:0] (shifted) for rxcount register - 0 for TX
 */
void st_usbfs_assign_buffer(uint16_t ep_id, uint32_t dir_tx, uint16_t *ram_ofs, uint16_t rx_blocks) {
	uint16_t ofs = (*ram_ofs + 3) & ~3;
	*ram_ofs = ofs;
	if(dir_tx)
		*USB_CHEP_TXRXBD(ep_id) = ofs;
	else
		*USB_CHEP_RXTXBD(ep_id) = (rx_blocks << 16) | ofs;
}

// NOTE: this seems to be sorta ok? (byteorder fine also)
void st_usbfs_copy_to_pm(uint16_t ep_id, const void *buf, uint16_t len)
{
	uint32_t buf_ofs = *USB_CHEP_TXRXBD(ep_id) & CHEP_BD_ADDR_MASK;
	volatile uint32_t *PM = (volatile uint32_t *) (USB_PMA_BASE + buf_ofs);
	const uint8_t *lbuf = buf;
	uint32_t i,v;

	for (v=i=0; i<len; i++, lbuf++) {
		v<<=8;
		v|=*lbuf;
		if((i&3) == 3)
			*PM++ = __builtin_bswap32(v);
	}

	// remainder?
	i = i&3;
	if(i) {
		i = (4-i) << 3;
		*PM = __builtin_bswap32(v<<i);
	}

	*USB_CHEP_TXRXBD(ep_id) = (len << CHEP_BD_COUNT_SHIFT) | buf_ofs;
}

/**
 * Copy a data buffer from packet memory.
 *
 * @param ep_id Endpoint ID (0..7)
 * @param buf Destination pointer for data buffer.
 * @param len Number of bytes to copy.
 */
// NOTE: this seems to be sorta ok? (byteorder fine also)
uint16_t st_usbfs_copy_from_pm(uint16_t ep_id, void *buf, uint16_t len)
{
	uint32_t v, i, buf_desc = *USB_CHEP_RXTXBD(ep_id);
	uint32_t count = (buf_desc >> CHEP_BD_COUNT_SHIFT) & CHEP_BD_COUNT_MASK;
	volatile uint32_t *PM = (volatile uint32_t *) (USB_PMA_BASE + (buf_desc & CHEP_BD_ADDR_MASK));
	uint8_t *dst = buf;

	count = MIN(count, len);
	for(v=i=0; i<count; i++, dst++, v>>=8) {
		if(!(i&3))
			v = *PM++;
		*dst = v&0xff;
	}
	return count;
}

static void st_usbfs_v3_disconnect(usbd_device *usbd_dev, bool disconnected)
{
	(void)usbd_dev;
	uint16_t reg = GET_REG(USB_BCDR_REG);
	if (disconnected) {
		SET_REG(USB_BCDR_REG, reg & ~USB_BCDR_DPPU);
	} else {
		SET_REG(USB_BCDR_REG, reg | USB_BCDR_DPPU);
	}
}

const struct _usbd_driver st_usbfs_v3_usb_driver = {
	.init = st_usbfs_v3_usbd_init,
	.set_address = st_usbfs_set_address,
	.ep_setup = st_usbfs_ep_setup,
	.ep_reset = st_usbfs_endpoints_reset,
	.ep_stall_set = st_usbfs_ep_stall_set,
	.ep_stall_get = st_usbfs_ep_stall_get,
	.ep_nak_set = st_usbfs_ep_nak_set,
	.ep_write_packet = st_usbfs_ep_write_packet,
	.ep_read_packet = st_usbfs_ep_read_packet,
	.disconnect = st_usbfs_v3_disconnect,
	.poll = st_usbfs_poll,
};

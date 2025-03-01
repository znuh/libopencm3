/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2014 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
 * Copyright (C) 2025 Benedikt Heinz <zn000h@gmail.com>
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
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include "../usb/usb_private.h"
#include "common/st_usbfs_core.h"

/** Initialize the USB device controller hardware of the STM32. */
static usbd_device *st_usbfs_v2_usbd_init(void)
{
	rcc_periph_clock_enable(RCC_USB);
	SET_REG(USB_CNTR_REG, 0);
	SET_REG(USB_BTABLE_REG, USB_BTABLE_OFS);
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
	if(dir_tx)
		USB_SET_EP_TX_ADDR(ep_id, *ram_ofs);
	else {
		USB_SET_EP_RX_ADDR(ep_id, *ram_ofs);
		USB_SET_EP_RX_COUNT(ep_id, rx_blocks);
	}
}

void st_usbfs_copy_to_pm(uint16_t ep_id, const void *buf, uint16_t len)
{
	/*
	 * This is a bytewise copy, so it always works, even on CM0(+)
	 * that don't support unaligned accesses.
	 */
	const uint8_t *lbuf = buf;
	volatile uint16_t *PM = (volatile void *)USB_GET_EP_TX_BUFF(ep_id);
	uint32_t n_words = len >> 1;

	/* copy complete words */
	for(;n_words;n_words--,lbuf+=2) {
		*PM++ = (uint16_t)lbuf[1] << 8 | *lbuf;
	}

	/* copy remaining byte if odd length */
	if(len&1)
		*PM = *lbuf;

	USB_SET_EP_TX_COUNT(ep_id, len);
}

/**
 * Copy a data buffer from packet memory.
 *
 * @param ep_id Endpoint ID (0..7)
 * @param buf Destination pointer for data buffer.
 * @param len Number of bytes to copy.
 */
uint16_t st_usbfs_copy_from_pm(uint16_t ep_id, void *buf, uint16_t len)
{
	const volatile uint16_t *PM = (volatile void *)USB_GET_EP_RX_BUFF(ep_id);
	uint16_t res = MIN(USB_GET_EP_RX_COUNT(ep_id) & 0x3ff, len);
	uint8_t odd = res & 1;
	len = res >> 1;

	if (((uintptr_t) buf) & 0x01) {
		for (; len; PM++, len--) {
			uint16_t value = *PM;
			*(uint8_t *) buf++ = value;
			*(uint8_t *) buf++ = value >> 8;
		}
	} else {
		for (; len; PM++, buf += 2, len--) {
			*(uint16_t *) buf = *PM;
		}
	}

	if (odd) {
		*(uint8_t *) buf = *(uint8_t *) PM;
	}
	return res;
}

static void st_usbfs_v2_disconnect(usbd_device *usbd_dev, bool disconnected)
{
	(void)usbd_dev;
	uint16_t reg = GET_REG(USB_BCDR_REG);
	if (disconnected) {
		SET_REG(USB_BCDR_REG, reg & ~USB_BCDR_DPPU);
	} else {
		SET_REG(USB_BCDR_REG, reg | USB_BCDR_DPPU);
	}
}

const struct _usbd_driver st_usbfs_v2_usb_driver = {
	.init = st_usbfs_v2_usbd_init,
	.set_address = st_usbfs_set_address,
	.ep_setup = st_usbfs_ep_setup,
	.ep_reset = st_usbfs_endpoints_reset,
	.ep_stall_set = st_usbfs_ep_stall_set,
	.ep_stall_get = st_usbfs_ep_stall_get,
	.ep_nak_set = st_usbfs_ep_nak_set,
	.ep_write_packet = st_usbfs_ep_write_packet,
	.ep_read_packet = st_usbfs_ep_read_packet,
	.disconnect = st_usbfs_v2_disconnect,
	.poll = st_usbfs_poll,
};

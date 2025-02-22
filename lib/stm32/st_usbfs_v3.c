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
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include "../usb/usb_private.h"
#include "common/st_usbfs_core.h"

/** Initialize the USB device controller hardware of the STM32. */
static usbd_device *st_usbfs_v3_usbd_init(void)
{
	uint32_t t_startup = 32;

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
	SET_REG(USB_BTABLE_REG, 0);
	SET_REG(USB_ISTR_REG, 0);

	/* Enable RESET, SUSPEND, RESUME and CTR interrupts. */
	SET_REG(USB_CNTR_REG, USB_CNTR_RESETM | USB_CNTR_CTRM |
		USB_CNTR_SUSPM | USB_CNTR_WKUPM);
	SET_REG(USB_BCDR_REG, USB_BCDR_DPPU);
	return &st_usbfs_dev;
}

void st_usbfs_copy_to_pm(volatile void *vPM, const void *buf, uint16_t len)
{
	const uint16_t *lbuf = buf;
	volatile uint32_t *PM = vPM;
	for (len = (len + 1) >> 1; len; len--) {
		*PM++ = *lbuf++;
	}
}

/**
 * Copy a data buffer from packet memory.
 *
 * @param buf Source pointer to data buffer.
 * @param vPM Destination pointer into packet memory.
 * @param len Number of bytes to copy.
 */
void st_usbfs_copy_from_pm(void *buf, const volatile void *vPM, uint16_t len)
{
	uint16_t *lbuf = buf;
	const volatile uint16_t *PM = vPM;
	uint8_t odd = len & 1;

	for (len >>= 1; len; PM += 2, lbuf++, len--) {
		*lbuf = *PM;
	}

	if (odd) {
		*(uint8_t *) lbuf = *(uint8_t *) PM;
	}
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

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include <stdio.h>
#include "usb-gadget0.h"

/* no trace on cm0 #define ER_DEBUG */
#ifdef ER_DEBUG
#define ER_DPRINTF(fmt, ...) \
	do { printf(fmt, ## __VA_ARGS__); } while (0)
#else
#define ER_DPRINTF(fmt, ...) \
	do { } while (0)
#endif

#include "trace.h"
void trace_send_blocking8(int stimulus_port, char c)
{
	(void)stimulus_port;
	(void)c;
}

/* MCU does not have a TIM6 */
void delay_setup(void);
void delay_setup(void) {}

/* use dummy delay function for now */
void delay_us(uint16_t us);
void delay_us(uint16_t us) {(void)us;}

int main(void)
{
	rcc_clock_setup_in_hsi48_out_48mhz();
	crs_autotrim_usb_enable();
	rcc_set_usbclk_source(RCC_HSIUSB48);

	usbd_device *usbd_dev = gadget0_init(&st_usbfs_usb_driver,
					     "stm32c071-generic");

	ER_DPRINTF("bootup complete\n");
	while (1) {
		gadget0_run(usbd_dev);
	}

}

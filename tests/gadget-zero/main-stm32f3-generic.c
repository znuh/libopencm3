/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
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

/* "generic" could be any board, but this file is pre-configured for 
 * a board with a 12MHz HSE and a STM32F302K8U6 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include "usb-gadget0.h"

static const struct rcc_clock_scale rcc_hse12mhz_72mhz = {
	.pllsrc = RCC_CFGR_PLLSRC_HSE_PREDIV,
	.pllmul = RCC_CFGR_PLLMUL_MUL6,
	.plldiv = RCC_CFGR2_PREDIV_NODIV,
	.usbdiv1 = false,
	.flash_waitstates = 2,
	.hpre = RCC_CFGR_HPRE_NODIV,
	.ppre1 = RCC_CFGR_PPRE_DIV2,
	.ppre2 = RCC_CFGR_PPRE_NODIV,
	.ahb_frequency = 72e6,
	.apb1_frequency = 36e6,
	.apb2_frequency = 72e6,
};

int main(void)
{
	rcc_clock_setup_pll(&rcc_hse12mhz_72mhz);

	rcc_periph_clock_enable(RCC_GPIOA);

	/* configure PA11/PA12 for USB AF */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11|GPIO12);
	gpio_set_af(GPIOA, GPIO_AF14, GPIO11|GPIO12);

	usbd_device *usbd_dev = gadget0_init(&st_usbfs_usb_driver,
					     "stm32f3-generic");

	while (1)
		gadget0_run(usbd_dev);
}


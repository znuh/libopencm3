/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2015 Robin Kreis <r.kreis@uni-bremen.de>
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

/*
 * This is a "private" header file for usbd implementations.
 * As opposed to the "public" headers under include that describe the hardware,
 * this is purely an implementation detail of usbd drivers.
 */

#ifndef ST_USBFS_CORE
#define ST_USBFS_CORE

#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>

#define USBD_PM_TOP 0x40

void st_usbfs_set_address(usbd_device *dev, uint8_t addr);

void st_usbfs_ep_setup(usbd_device *usbd_dev, uint8_t addr,
		uint8_t type, uint16_t max_size,
		void (*callback) (usbd_device *usbd_dev,
		uint8_t ep));

void st_usbfs_endpoints_reset(usbd_device *usbd_dev);
void st_usbfs_ep_stall_set(usbd_device *usbd_dev, uint8_t addr, uint8_t stall);
uint8_t st_usbfs_ep_stall_get(usbd_device *usbd_dev, uint8_t addr);
void st_usbfs_ep_nak_set(usbd_device *usbd_dev, uint8_t addr, uint8_t nak);
uint16_t st_usbfs_ep_write_packet(usbd_device *usbd_dev, uint8_t addr,
				  const void *buf, uint16_t len);
uint16_t st_usbfs_ep_read_packet(usbd_device *usbd_dev, uint8_t addr,
				 void *buf, uint16_t len);
void st_usbfs_poll(usbd_device *usbd_dev);

/* These must be implemented by the device specific driver */

struct st_usbfs_pm_s {
	void (*assign_buffer)(uint16_t ep_id, uint32_t dir_tx, uint16_t *ram_ofs, uint16_t rx_blocks);
	void (*write)(uint16_t ep_id, const void *vsrc, uint16_t len);
	uint16_t (*read)(uint16_t ep_id, void *vdst, uint16_t len);
};

#endif

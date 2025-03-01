/** @addtogroup usb_defines
 */
/*
 * This file is part of the libopencm3 project.
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

/* THIS FILE SHOULD NOT BE INCLUDED DIRECTLY !
 * Use top-level <libopencm3/stm32/st_usbfs.h>
 *
 * Additional definitions for F0 devices :
 * -F0x0 (RM0360),
 * -F04x, F0x2, F0x8 (RM0091)
 */

/** @cond */
#ifdef LIBOPENCM3_ST_USBFS_H
/** @endcond */
#ifndef LIBOPENCM3_ST_USBFS_V2_H
#define LIBOPENCM3_ST_USBFS_V2_H

#include <libopencm3/stm32/common/st_usbfs_common.h>

/* include additional registers (LPM and BCD) for USB v2 peripheral */
#include <libopencm3/stm32/common/st_usbfs_regs_v2.h>

/*****************************************************************************/
/* Module definitions                                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Register definitions                                                      */
/*****************************************************************************/

/* USB Buffer table address register */
#define USB_BTABLE_REG		(&MMIO32(USB_DEV_FS_BASE + 0x50))

/*****************************************************************************/
/* Register values                                                           */
/*****************************************************************************/

/* --- USB BTABLE registers ------------------------------------------------ */

#define USB_GET_BTABLE		GET_REG16(USB_BTABLE_REG)

/* --- USB BTABLE manipulators --------------------------------------------- */

#define USB_GET_EP_TX_ADDR(EP)		GET_REG16(USB_EP_TX_ADDR(EP))
#define USB_GET_EP_TX_COUNT(EP)		GET_REG16(USB_EP_TX_COUNT(EP))
#define USB_GET_EP_RX_ADDR(EP)		GET_REG16(USB_EP_RX_ADDR(EP))
#define USB_GET_EP_RX_COUNT(EP)		GET_REG16(USB_EP_RX_COUNT(EP))
#define USB_SET_EP_TX_ADDR(EP, ADDR)	SET_REG16(USB_EP_TX_ADDR(EP), ADDR)
#define USB_SET_EP_TX_COUNT(EP, COUNT)	SET_REG16(USB_EP_TX_COUNT(EP), COUNT)
#define USB_SET_EP_RX_ADDR(EP, ADDR)	SET_REG16(USB_EP_RX_ADDR(EP), ADDR)
#define USB_SET_EP_RX_COUNT(EP, COUNT)	SET_REG16(USB_EP_RX_COUNT(EP), COUNT)

/* --- USB BTABLE registers ------------------------------------------------ */

#define USB_EP_TX_ADDR(EP) \
	((uint16_t *)(USB_PMA_BASE + (USB_GET_BTABLE + (EP) * 8 + 0) * 1))

#define USB_EP_TX_COUNT(EP) \
	((uint16_t *)(USB_PMA_BASE + (USB_GET_BTABLE + (EP) * 8 + 2) * 1))

#define USB_EP_RX_ADDR(EP) \
	((uint16_t *)(USB_PMA_BASE + (USB_GET_BTABLE + (EP) * 8 + 4) * 1))

#define USB_EP_RX_COUNT(EP) \
	((uint16_t *)(USB_PMA_BASE + (USB_GET_BTABLE + (EP) * 8 + 6) * 1))

/* --- USB BTABLE manipulators --------------------------------------------- */

#define USB_GET_EP_TX_BUFF(EP) \
	(USB_PMA_BASE + (uint8_t *)(USB_GET_EP_TX_ADDR(EP) * 1))

#define USB_GET_EP_RX_BUFF(EP) \
	(USB_PMA_BASE + (uint8_t *)(USB_GET_EP_RX_ADDR(EP) * 1))

#define	ST_USBFS_DRIVER			&st_usbfs_v2_usb_driver

#endif
/** @cond */
#else
#error "st_usbfs_v2.h should not be included directly, only via st_usbfs.h"
#endif
/** @endcond */


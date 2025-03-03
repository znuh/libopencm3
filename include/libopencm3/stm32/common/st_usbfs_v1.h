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
 * Additional definitions for F1, F3, L1 devices:
 * -F102, F103 (RM0008)
 * -F302x{B,C}; *NOT* F302x{6,8,D,E} !! (USB_BTABLE access issues) (RM0365)
 * -F303x{B,C}; *NOT* F303x{D,E} !! (USB_BTABLE access issues) (RM0316)
 * -F37x (RM0313)
 * -L1xx (RM0038)
 */

/** @cond */
#ifdef LIBOPENCM3_ST_USBFS_H
/** @endcond */
#ifndef LIBOPENCM3_ST_USBFS_V1_H
#define LIBOPENCM3_ST_USBFS_V1_H

#include <libopencm3/stm32/common/st_usbfs_common.h>

/* enable and include BTADDR register for USB v1 peripheral */
#define ST_USBFS_HAVE_BTADDR
#include <libopencm3/stm32/common/st_usbfs_ext.h>

/*****************************************************************************/
/* Register definitions                                                      */
/*****************************************************************************/

/* Dedicated packet buffer memory SRAM access scheme: 1 x 16 bits / word (see RM) */

#define USB_BT32_GET(OFS)			GET_REG32(USB_PMA_BASE + ((OFS)<<1))
#define USB_BT32_SET(OFS, VAL)		SET_REG32(USB_PMA_BASE + ((OFS)<<1), VAL)

/* --- USB BTABLE Registers ------------------------------------------------ */

#define BT_TX_ADDR(EP)				(USB_BTABLE_OFS + (EP) * 8 + 0)
#define BT_TX_COUNT(EP)				(USB_BTABLE_OFS + (EP) * 8 + 2)
#define BT_RX_ADDR(EP)				(USB_BTABLE_OFS + (EP) * 8 + 4)
#define BT_RX_COUNT(EP)				(USB_BTABLE_OFS + (EP) * 8 + 6)

#define	ST_USBFS_DRIVER				&st_usbfs_v1_usb_driver

#endif
/** @cond */
#else
#error "st_usbfs_v1.h should not be included directly, only via st_usbfs.h"
#endif
/** @endcond */


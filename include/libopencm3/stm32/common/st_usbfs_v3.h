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
 */

/** @cond */
#ifdef LIBOPENCM3_ST_USBFS_H
/** @endcond */
#ifndef LIBOPENCM3_ST_USBFS_V3_H
#define LIBOPENCM3_ST_USBFS_V3_H

#include <libopencm3/stm32/common/st_usbfs_common.h>

/* include additional registers (LPM and BCD) inherited from USB v2 peripheral */
#include <libopencm3/stm32/common/st_usbfs_regs_v2.h>

/*****************************************************************************/
/* Module definitions                                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Register definitions                                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Register values                                                           */
/*****************************************************************************/

/* --- Channel/endpoint buffer descriptors  -------------------------------- */

#define USB_CHEP_TXRXBD(EP) \
	((uint32_t *)(USB_PMA_BASE + ((EP) * 8 + 0) * 1))

#define CHEP_BD_ADDR_MASK 				0xffff
#define CHEP_BD_COUNT_SHIFT				16
#define CHEP_BD_COUNT_MASK				0x3ff

#define USB_CHEP_RXTXBD(EP) \
	((uint32_t *)(USB_PMA_BASE + ((EP) * 8 + 4) * 1))

#define	ST_USBFS_DRIVER					&st_usbfs_v3_usb_driver

#endif
/** @cond */
#else
#error "st_usbfs_v3.h should not be included directly, only via st_usbfs.h"
#endif
/** @endcond */

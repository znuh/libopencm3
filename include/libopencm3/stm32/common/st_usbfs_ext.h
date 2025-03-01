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

/** @cond */
#ifdef LIBOPENCM3_ST_USBFS_H
/** @endcond */
#ifndef LIBOPENCM3_ST_USBFS_EXT_H
#define LIBOPENCM3_ST_USBFS_EXT_H

/* This file defines additional registers and Bits added/removed in different
 * versions of the USB peripheral.
 */

#include <libopencm3/stm32/common/st_usbfs_common.h>

/*****************************************************************************/
/* Module definitions                                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Register definitions                                                      */
/*****************************************************************************/

#ifdef ST_USBFS_HAVE_LPM
#define USB_LPMCSR_REG		(&MMIO32(USB_DEV_FS_BASE + 0x54))
#endif

#ifdef ST_USBFS_HAVE_BCD
#define USB_BCDR_REG		(&MMIO32(USB_DEV_FS_BASE + 0x58))
#endif

#ifdef ST_USBFS_HAVE_BTADDR
/* USB Buffer table address register */
#define USB_BTABLE_REG		(&MMIO32(USB_DEV_FS_BASE + 0x50))
#endif

/*****************************************************************************/
/* Register values                                                           */
/*****************************************************************************/

/* --- USB control register masks / bits ----------------------------------- */

#ifdef ST_USBFS_HAVE_LPM
#define USB_CNTR_L1REQM		(1 << 7)
#define USB_CNTR_L1RESUME	(1 << 5)

/* --- USB interrupt status register masks / bits -------------------------- */

#define USB_ISTR_L1REQ		(1 << 7)

/* --- LPM control and status register USB_LPMCSR Values --------------------*/

#define USB_LPMCSR_BESL_SHIFT	4
#define USB_LPMCSR_BESL		(15 << USB_LPMCSR_BESL_SHIFT)

#define USB_LPMCSR_REMWAKE	(1 << 3)
#define USB_LPMCSR_LPMACK	(1 << 1)
#define USB_LPMCSR_LPMEN	(1 << 0)
#endif

#ifdef ST_USBFS_HAVE_BCD
/* --- Battery Charging Detector Values ----------------------------------------------------------*/
#define USB_BCDR_DPPU		(1 << 15)
#define USB_BCDR_PS2DET		(1 << 7)
#define USB_BCDR_SDET		(1 << 6)
#define USB_BCDR_PDET		(1 << 5)
#define USB_BCDR_DCDET		(1 << 4)
#define USB_BCDR_SDEN		(1 << 3)
#define USB_BCDR_PDEN		(1 << 2)
#define USB_BCDR_DCDEN		(1 << 1)
#define USB_BCDR_BCDEN		(1 << 0)
#endif

#ifdef ST_USBFS_HAVE_BTADDR
/* --- USB BTABLE registers ------------------------------------------------ */
#define USB_GET_BTABLE		GET_REG16(USB_BTABLE_REG)
#define USB_BTABLE_OFS		0	/* place BTABLE at start of packet RAM */
#endif

#endif
/** @cond */
#else
#error "st_usbfs_regs_v2.h should not be included directly, only via st_usbfs.h"
#endif
/** @endcond */


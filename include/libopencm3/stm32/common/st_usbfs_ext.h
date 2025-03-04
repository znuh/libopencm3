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
#define USB_GET_BTABLE				GET_REG16(USB_BTABLE_REG)
#endif

/* --- BTABLE offsets ------------------------------------------------ */
#define USB_BTABLE_OFS				0	/* BTABLE resides at start of packet RAM */

#define BT_TX_ADDR(EP)				(USB_BTABLE_OFS + (EP) * 8 + 0)
#define BT_TX_COUNT(EP)				(USB_BTABLE_OFS + (EP) * 8 + 2)
#define BT_RX_ADDR(EP)				(USB_BTABLE_OFS + (EP) * 8 + 4)
#define BT_RX_COUNT(EP)				(USB_BTABLE_OFS + (EP) * 8 + 6)

/* --- USB Packet Memory (PMA) Access Schemes ------------------------------- */

#ifdef ST_USBFS_PMA_AS_1X16
/* Dedicated packet buffer memory SRAM access scheme: 1 x 16 bits / word (see RM) */
#define USB_BT32_GET(OFS)			GET_REG32(USB_PMA_BASE + ((OFS)<<1))
#define USB_BT32_SET(OFS, VAL)		SET_REG32(USB_PMA_BASE + ((OFS)<<1), VAL)
#endif /* ST_USBFS_PMA_AS_1X16 */

#ifdef ST_USBFS_PMA_AS_2X16
/* Dedicated packet buffer memory SRAM access scheme: 2 x 16 bits / word (see RM) */
#define USB_BT16_GET(OFS)			GET_REG16(USB_PMA_BASE + (OFS))
#define USB_BT16_SET(OFS, VAL)		SET_REG16(USB_PMA_BASE + (OFS), VAL)
#endif /* ST_USBFS_PMA_AS_2X16 */

#ifdef ST_USBFS_PMA_AS_1X32
/* Dedicated packet buffer memory SRAM access scheme: 32 bits (see RM)
 * This new peripheral version usually(?) does not have a BTADDR register anymore.
 * The BT address is fixed zero. */
#define USB_CHEP_TXRXBD(EP) 		((uint32_t *)(USB_PMA_BASE + ((EP) * 8 + 0) * 1))
#define USB_CHEP_RXTXBD(EP) 		((uint32_t *)(USB_PMA_BASE + ((EP) * 8 + 4) * 1))
#define CHEP_BD_ADDR_MASK 			0xffff
#define CHEP_BD_COUNT_SHIFT			16
#define CHEP_BD_COUNT_MASK			0x3ff
#endif /* ST_USBFS_PMA_AS_1X32 */

#if (defined(ST_USBFS_PMA_AS_1X16) && defined(ST_USBFS_PMA_AS_2X16)) \
	|| (defined(ST_USBFS_PMA_AS_1X16) && defined(ST_USBFS_PMA_AS_1X32)) \
	|| (defined(ST_USBFS_PMA_AS_2X16) && defined(ST_USBFS_PMA_AS_1X32))
#define PMA_AS_MULTIPLE
#endif

#endif
/** @cond */
#else
#error "st_usbfs_regs_v2.h should not be included directly, only via st_usbfs.h"
#endif
/** @endcond */


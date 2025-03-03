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

/* enable and include additional registers (LPM and BCD) for USB v3 peripheral */
#define ST_USBFS_HAVE_LPM
#define ST_USBFS_HAVE_BCD

/* Dedicated packet buffer memory SRAM access scheme: 32 bits (see RM) */
#define ST_USBFS_PMA_AS_1X32

#include <libopencm3/stm32/common/st_usbfs_ext.h>

#define	ST_USBFS_DRIVER					&st_usbfs_v3_usb_driver

#endif
/** @cond */
#else
#error "st_usbfs_v3.h should not be included directly, only via st_usbfs.h"
#endif
/** @endcond */

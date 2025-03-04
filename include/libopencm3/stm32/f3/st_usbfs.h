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

#ifndef LIBOPENCM3_ST_USBFS_H
#	error Do not include directly !
#else

/* F302x{6,8,D,E} and F303x{D,E} use a 2x16 PMA access scheme instead of the
 * classic 1x16. We enable this access scheme here additionally.
 * 1x16 PMA access scheme for the other F302/F303 is enabled by default in st_usbfs_v1.h */
#define ST_USBFS_PMA_AS_2X16

#include <libopencm3/stm32/common/st_usbfs_v1.h>

#endif

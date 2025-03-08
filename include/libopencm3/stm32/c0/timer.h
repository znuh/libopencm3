/** @defgroup timer_defines Timer Defines
 *
 * @ingroup STM32C0xx_defines
 *
 * @brief <b>Defined Constants and Types for the STM32C0xx Timers</b>
 *
 * @version 1.0.0
 *
 * LGPL License Terms @ref lgpl_license
 *  */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2019 Guillaume Revaillot <g.revaillot@gmail.com>
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

#pragma once
#include <libopencm3/stm32/common/timer_common_all.h>

/**@{*/

/* Alternate Function (TIMx_AF1) */
#define TIM_AF1(tim_base)		MMIO32((tim_base) + 0x60)
#define TIM1_AF1			TIM_AF1(TIM1)
#define TIM2_AF1			TIM_AF1(TIM2)
#define TIM3_AF1			TIM_AF1(TIM3)
#define TIM15_AF1			TIM_AF1(TIM15)
#define TIM16_AF1			TIM_AF1(TIM16)
#define TIM17_AF1			TIM_AF1(TIM17)

/* Alternate Function (TIMx_AF2) */
#define TIM_AF2(tim_base)		MMIO32((tim_base) + 0x64)
#define TIM1_AF2			TIM_AF2(TIM1)

/* Input Selection Register (TIMx_TISEL) */
#define TIM_TISEL(tim_base)		MMIO32((tim_base) + 0x68)
#define TIM1_TISEL			TIM_TISEL(TIM1)
#define TIM2_TISEL			TIM_TISEL(TIM2)
#define TIM3_TISEL			TIM_TISEL(TIM3)
#define TIM14_TISEL			TIM_TISEL(TIM14)
#define TIM15_TISEL			TIM_TISEL(TIM15)
#define TIM16_TISEL			TIM_TISEL(TIM16)
#define TIM17_TISEL			TIM_TISEL(TIM17)

BEGIN_DECLS

END_DECLS

/**@}*/

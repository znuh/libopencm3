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

#ifndef LIBOPENCM3_MEMORYMAP_H
#define LIBOPENCM3_MEMORYMAP_H

#include <libopencm3/cm3/memorymap.h>

#define FLASH_BASE			(0x08000000U)
#define PERIPH_BASE			(0x40000000U)
#define IOPORT_BASE			(0x50000000U)
#define INFO_BASE			(0x1fff7500U)
#define PERIPH_BASE_APB			(PERIPH_BASE + 0x00000)
#define PERIPH_BASE_AHB			(PERIPH_BASE + 0x20000)

/* APB */
#define TIM2_BASE			(PERIPH_BASE_APB + 0x0000)
#define TIM3_BASE			(PERIPH_BASE_APB + 0x0400)
#define TIM14_BASE			(PERIPH_BASE_APB + 0x2000)
#define RTC_BASE			(PERIPH_BASE_APB + 0x2800)
#define WWDG_BASE			(PERIPH_BASE_APB + 0x2c00)
#define IWDG_BASE			(PERIPH_BASE_APB + 0x3000)
#define SPI2_BASE			(PERIPH_BASE_APB + 0x3800)
#define USART2_BASE			(PERIPH_BASE_APB + 0x4400)
#define USART3_BASE			(PERIPH_BASE_APB + 0x4800)
#define USART4_BASE			(PERIPH_BASE_APB + 0x4C00)
#define I2C1_BASE			(PERIPH_BASE_APB + 0x5400)
#define I2C2_BASE			(PERIPH_BASE_APB + 0x5800)
#define USB_DEV_FS_BASE		(PERIPH_BASE_APB + 0x5c00)
#define FDCAN1_BASE			(PERIPH_BASE_APB + 0x6400)
#define CRS_BASE			(PERIPH_BASE_APB + 0x6c00)
#define POWER_CONTROL_BASE		(PERIPH_BASE_APB + 0x7000)
#define USB_PMA_BASE			(PERIPH_BASE_APB + 0x9800)
#define USB_PMA_SIZE 		2048
#define FDCANMSG_BASE			(PERIPH_BASE_APB + 0xB400)
#define SYSCFG_BASE			(PERIPH_BASE_APB + 0x10000)
#define SYSCFG_ITLINE_BASE		(PERIPH_BASE_APB + 0x10080)
#define ADC1_BASE			(PERIPH_BASE_APB + 0x12400)
#define TIM1_BASE			(PERIPH_BASE_APB + 0x12C00)
#define SPI1_BASE			(PERIPH_BASE_APB + 0x13000)
#define USART1_BASE			(PERIPH_BASE_APB + 0x13800)
#define TIM15_BASE			(PERIPH_BASE_APB + 0x14000)
#define TIM16_BASE			(PERIPH_BASE_APB + 0x14400)
#define TIM17_BASE			(PERIPH_BASE_APB + 0x14800)
#define DBGMCU_BASE			(PERIPH_BASE_APB + 0x15800)

/* AHB */
#define DMA1_BASE			(PERIPH_BASE_AHB + 0x00000)
#define DMAMUX_BASE			(PERIPH_BASE_AHB + 0x00800)
#define RCC_BASE			(PERIPH_BASE_AHB + 0x01000)
#define EXTI_BASE			(PERIPH_BASE_AHB + 0x01800)
#define FLASH_MEM_INTERFACE_BASE	(PERIPH_BASE_AHB + 0x02000)
#define CRC_BASE			(PERIPH_BASE_AHB + 0x03000)

#define GPIO_PORT_A_BASE		(IOPORT_BASE + 0x00000)
#define GPIO_PORT_B_BASE		(IOPORT_BASE + 0x00400)
#define GPIO_PORT_C_BASE		(IOPORT_BASE + 0x00800)
#define GPIO_PORT_D_BASE		(IOPORT_BASE + 0x00c00)
#define GPIO_PORT_F_BASE		(IOPORT_BASE + 0x01400)

/* Device Electronic Signature */
#define DESIG_FLASH_SIZE_BASE		(0x1FFF75A0)
#define DESIG_UNIQUE_ID_BASE		(0x1FFF7550)
#define DESIG_UNIQUE_ID0		MMIO32(DESIG_UNIQUE_ID_BASE)
#define DESIG_UNIQUE_ID1		MMIO32(DESIG_UNIQUE_ID_BASE + 4)
#define DESIG_UNIQUE_ID2		MMIO32(DESIG_UNIQUE_ID_BASE + 8)

/* ST provided factory calibration values @ 3.0V */
#define ST_VREFINT_CAL			MMIO16((INFO_BASE + 0x6A))
#define ST_TSENSE_CAL1_30C		MMIO16((INFO_BASE + 0x68))

/* Option Bytes */
#define OPTION_BYTES_BASE			(0x1FFF7800)
#define FLASH_OPTION_BYTES			MMIO32(OPTION_BYTES_BASE)

#endif

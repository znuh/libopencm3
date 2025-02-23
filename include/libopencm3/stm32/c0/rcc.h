/** @defgroup rcc_defines RCC Defines
 *
 * @ingroup STM32C0xx_defines
 *
 * @brief <b>Defined Constants and Types for the STM32C0xx Reset and Clock Control</b>
 *
 * @version 1.0.0
 *
 * LGPL License Terms @ref lgpl_license
 *  */
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
 *
 */

/**@{*/

#ifndef LIBOPENCM3_RCC_H
#define LIBOPENCM3_RCC_H

#include <libopencm3/stm32/pwr.h>

/** @defgroup rcc_registers Reset and Clock Control Register
@{*/
#define RCC_CR				MMIO32(RCC_BASE + 0x00)
#define RCC_ICSCR			MMIO32(RCC_BASE + 0x04)
#define RCC_CFGR			MMIO32(RCC_BASE + 0x08)
#define RCC_CRRCR			MMIO32(RCC_BASE + 0x14)
#define RCC_CIER			MMIO32(RCC_BASE + 0x18)
#define RCC_CIFR			MMIO32(RCC_BASE + 0x1c)
#define RCC_CICR			MMIO32(RCC_BASE + 0x20)
#define RCC_IOPRSTR_OFFSET			0x24
#define RCC_IOPRSTR					MMIO32(RCC_BASE + RCC_IOPRSTR_OFFSET)
#define RCC_AHBRSTR_OFFSET			0x28
#define RCC_AHBRSTR					MMIO32(RCC_BASE + RCC_AHBRSTR_OFFSET)
#define RCC_APBRSTR1_OFFSET			0x2c
#define RCC_APBRSTR1				MMIO32(RCC_BASE + RCC_APBRSTR1_OFFSET)
#define RCC_APBRSTR2_OFFSET			0x30
#define RCC_APBRSTR2				MMIO32(RCC_BASE + RCC_APBRSTR2_OFFSET)
#define RCC_IOPENR_OFFSET			0x34
#define RCC_IOPENR					MMIO32(RCC_BASE + RCC_IOPENR_OFFSET)
#define RCC_AHBENR_OFFSET			0x38
#define RCC_AHBENR					MMIO32(RCC_BASE + RCC_AHBENR_OFFSET)
#define RCC_APBENR1_OFFSET			0x3c
#define RCC_APBENR1					MMIO32(RCC_BASE + RCC_APBENR1_OFFSET)
#define RCC_APBENR2_OFFSET			0x40
#define RCC_APBENR2					MMIO32(RCC_BASE + RCC_APBENR2_OFFSET)
#define RCC_IOPSMENR_OFFSET			0x44
#define RCC_IOPSMENR				MMIO32(RCC_BASE + RCC_IOPSMENR_OFFSET)
#define RCC_AHBSMENR_OFFSET			0x48
#define RCC_AHBSMENR				MMIO32(RCC_BASE + RCC_AHBSMENR_OFFSET)
#define RCC_APBSMENR1_OFFSET		0x4c
#define RCC_APBSMENR1				MMIO32(RCC_BASE + RCC_APBSMENR1_OFFSET)
#define RCC_APBSMENR2_OFFSET		0x50
#define RCC_APBSMENR2				MMIO32(RCC_BASE + RCC_APBSMENR2_OFFSET)
#define RCC_CCIPR			MMIO32(RCC_BASE + 0x54)
#define RCC_CCIPR2			MMIO32(RCC_BASE + 0x58)
#define RCC_CSR1			MMIO32(RCC_BASE + 0x5C)
#define RCC_CSR2			MMIO32(RCC_BASE + 0x60)
/**@}*/

/** @defgroup rcc_cr CR Clock control Register
@{*/
#define RCC_CR_HSIUSB48RDY		(1 << 23)
#define RCC_CR_HSIUSB48ON		(1 << 22)
#define RCC_CR_CSSON			(1 << 19)
#define RCC_CR_HSEBYP			(1 << 18)
#define RCC_CR_HSERDY			(1 << 17)
#define RCC_CR_HSEON			(1 << 16)

#define RCC_CR_HSIDIV_SHIFT		11
#define RCC_CR_HSIDIV_MASK		0x7
/** @defgroup rcc_cr_hsidiv HSI Div
 * @brief Division factor of the HSI16 oscillator to produce HSISYS clock
@sa rcc_cr_hsidiv
@{*/
#define RCC_CR_HSIDIV_DIV1		0
#define RCC_CR_HSIDIV_DIV2		1
#define RCC_CR_HSIDIV_DIV4		2
#define RCC_CR_HSIDIV_DIV8		3
#define RCC_CR_HSIDIV_DIV16		4
#define RCC_CR_HSIDIV_DIV32		5
#define RCC_CR_HSIDIV_DIV64		6
#define RCC_CR_HSIDIV_DIV128	7
/**@}*/

#define RCC_CR_HSIRDY			(1 << 10)
#define RCC_CR_HSIKERON			(1 << 9)
#define RCC_CR_HSION			(1 << 8)
/**@}*/

/* RCC TBD:
 * Bits 7:5 HSIKERDIV[2:0]: HSI48 kernel clock division factor
 * Bits 4:2 SYSDIV[2:0]: Clock division factor for system clock
 */


/** @defgroup rcc_icscr ICSCR Internal Clock Source Calibration Register
@{*/
#define RCC_ICSCR_HSITRIM_SHIFT		8
#define RCC_ICSCR_HSITRIM_MASK		0x1f
#define RCC_ICSCR_HSICAL_SHIFT		0
#define RCC_ICSCR_HSICAL_MASK		0xff
/**@}*/


/** @defgroup rcc_cfgr CFGR Configuration Register
@{*/
#define RCC_CFGR_MCOPRE_SHIFT	    28
#define RCC_CFGR_MCOPRE_MASK	    0x7
/** @defgroup rcc_cfgr_mcopre MCO Pre
 * @brief Division factor of microcontroler clock output
@sa rcc_cfgr_mcopre
@{*/
#define RCC_CFGR_MCOPRE_DIV1	    0
#define RCC_CFGR_MCOPRE_DIV2	    1
#define RCC_CFGR_MCOPRE_DIV4	    2
#define RCC_CFGR_MCOPRE_DIV8	    3
#define RCC_CFGR_MCOPRE_DIV16	    4
#define RCC_CFGR_MCOPRE_DIV32	    5
#define RCC_CFGR_MCOPRE_DIV64	    6
#define RCC_CFGR_MCOPRE_DIV128	    7
/**@}*/

#define RCC_CFGR_MCO_SHIFT			24
#define RCC_CFGR_MCO_MASK			0xf

/** @defgroup rcc_cfgr_mcosel MCO Sel
 * @brief Microcontroler clock output selector
@sa rcc_cfgr_mcosel
@{*/
#define RCC_CFGR_MCO_NOCLK			0x0
#define RCC_CFGR_MCO_SYSCLK			0x1
#define RCC_CFGR_MCO_HSI48			0x3
#define RCC_CFGR_MCO_HSE			0x4
#define RCC_CFGR_MCO_LSI			0x6
#define RCC_CFGR_MCO_LSE			0x7
#define RCC_CFGR_MCO_HSIUSB48		0x8

/**@}*/

#define RCC_CFGR_PPRE_MASK		0x7
#define RCC_CFGR_PPRE_SHIFT		12

/** @defgroup rcc_cfgr_ppre PPRE
 * @brief APB Prescaler
@sa rcc_cfgr_ppre
@{*/
#define RCC_CFGR_PPRE_NODIV		0x0
#define RCC_CFGR_PPRE_DIV2		0x4
#define RCC_CFGR_PPRE_DIV4		0x5
#define RCC_CFGR_PPRE_DIV8		0x6
#define RCC_CFGR_PPRE_DIV16		0x7
/**@}*/

#define RCC_CFGR_HPRE_MASK		0xf
#define RCC_CFGR_HPRE_SHIFT		8
/** @defgroup rcc_cfgr_hpre HPRE
 * @brief APB Prescaler
@sa rcc_cfgr_hpre
@{*/
#define RCC_CFGR_HPRE_NODIV		0x0
#define RCC_CFGR_HPRE_DIV2		0x8
#define RCC_CFGR_HPRE_DIV4		0x9
#define RCC_CFGR_HPRE_DIV8		0xa
#define RCC_CFGR_HPRE_DIV16		0xb
#define RCC_CFGR_HPRE_DIV64		0xc
#define RCC_CFGR_HPRE_DIV128	0xd
#define RCC_CFGR_HPRE_DIV256	0xe
#define RCC_CFGR_HPRE_DIV512	0xf
/**@}*/

#define RCC_CFGR_SWS_MASK			0x3
#define RCC_CFGR_SWS_SHIFT			3
/** @defgroup rcc_cfgr_sws SWS
 * @brief System clock switch status
@sa rcc_cfgr_sws
@{*/
#define RCC_CFGR_SWS_HSISYS			0x0
#define RCC_CFGR_SWS_HSE			0x1
#define RCC_CFGR_SWS_HSIUSB48		0x2
#define RCC_CFGR_SWS_LSI			0x3
#define RCC_CFGR_SWS_LSE			0x4
/**@}*/

#define RCC_CFGR_SW_MASK			0x3
#define RCC_CFGR_SW_SHIFT			0
/** @defgroup rcc_cfgr_sw SW
 * @brief System clock switch
@sa rcc_cfgr_sw
@{*/
#define RCC_CFGR_SW_HSISYS			0x0
#define RCC_CFGR_SW_HSE				0x1
#define RCC_CFGR_SW_HSIUSB48		0x2
#define RCC_CFGR_SW_LSI				0x3
#define RCC_CFGR_SW_LSE				0x4
/**@}*/
/**@}*/


/** @defgroup rcc_cier CIER Clock Interrupt Enable Register
@{*/
#define RCC_CIER_HSERDYIE			(1 << 4)
#define RCC_CIER_HSIRDYIE			(1 << 3)
#define RCC_CIER_HSIUSB48RDYIE		(1 << 2)
#define RCC_CIER_LSERDYIE			(1 << 1)
#define RCC_CIER_LSIRDYIE			(1 << 0)
/**@}*/

/** @defgroup rcc_cifr CIFR Clock Interrupt Flag Register
@{*/
#define RCC_CIFR_LSECSSF			(1 << 9)
#define RCC_CIFR_CSSF				(1 << 8)
#define RCC_CIFR_HSERDYF			(1 << 4)
#define RCC_CIFR_HSIRDYF			(1 << 3)
#define RCC_CIFR_HSIUSB48RDYF		(1 << 2)
#define RCC_CIFR_LSERDYF			(1 << 1)
#define RCC_CIFR_LSIRDYF			(1 << 0)
/**@}*/

/** @defgroup rcc_cicr CICR Clock Interrupt Clear Register
@{*/
#define RCC_CICR_LSECSSC			(1 << 9)
#define RCC_CICR_CSSC				(1 << 8)
#define RCC_CICR_HSERDYC			(1 << 4)
#define RCC_CICR_HSIRDYC			(1 << 3)
#define RCC_CICR_HSIUSB48RDYC		(1 << 2)
#define RCC_CICR_LSERDYC			(1 << 1)
#define RCC_CICR_LSIRDYC			(1 << 0)
/**@}*/

/** @defgroup rcc_ahbrstr_rst RCC_AHBRSTR reset values
@{*/
#define RCC_AHBRSTR_CRCRST			(1 << 12)
#define RCC_AHBRSTR_FLASHRST		(1 << 8)
#define RCC_AHBRSTR_DMA1RST			(1 << 0)
#define RCC_AHBRSTR_DMARST			RCC_AHBRSTR_DMA1RST
/**@}*/

/** @defgroup rcc_apb1rstr_rst RCC_APBRSTRx reset values (full set)
@{*/
/** @defgroup rcc_apbrstr1_rst RCC_APBRSTR1 reset values
@{*/
#define RCC_APBRSTR1_PWRRST				(1 << 28)
#define RCC_APBRSTR1_DBGRST				(1 << 27)
#define RCC_APBRSTR1_I2C2RST			(1 << 22)
#define RCC_APBRSTR1_I2C1RST			(1 << 21)
#define RCC_APBRSTR1_USART4RST			(1 << 19)
#define RCC_APBRSTR1_USART3RST			(1 << 18)
#define RCC_APBRSTR1_USART2RST			(1 << 17)
#define RCC_APBRSTR1_CRSRST			(1 << 16)
#define RCC_APBRSTR1_SPI2RST			(1 << 14)
#define RCC_APBRSTR1_USBRST			(1 << 13)
#define RCC_APBRSTR1_FDCANRST			(1 << 12)
#define RCC_APBRSTR1_TIM3RST			(1 << 1)
#define RCC_APBRSTR1_TIM2RST			(1 << 0)
/**@}*/

/** @defgroup rcc_apbrstr2_rst RCC_APBRSTR2 reset values
@{*/
#define RCC_APBRSTR2_ADCRST				(1 << 20)
#define RCC_APBRSTR2_TIM17RST			(1 << 18)
#define RCC_APBRSTR2_TIM16RST			(1 << 17)
#define RCC_APBRSTR2_TIM15RST			(1 << 16)
#define RCC_APBRSTR2_TIM14RST			(1 << 15)
#define RCC_APBRSTR2_USART1RST			(1 << 14)
#define RCC_APBRSTR2_SPI1RST			(1 << 12)
#define RCC_APBRSTR2_TIM1RST			(1 << 11)
#define RCC_APBRSTR2_SYSCFGRST			(1 << 0)
/**@}*/
/**@}*/

/** @defgroup rcc_ahbenr_en RCC_AHBENR enable values
@{*/
#define RCC_AHBENR_CRCEN			(1 << 12)
#define RCC_AHBENR_FLASHEN			(1 << 8)
#define RCC_AHBENR_DMA1EN			(1 << 0)
#define RCC_AHBENR_DMAEN			RCC_AHBENR_DMA1EN
/**@}*/

/** @defgroup rcc_apb1enr_en RCC_APBENRx enable values (full set)
@{*/
/** @defgroup rcc_apbenr1_en RCC_APBENR1 enable values
@{*/
#define RCC_APBENR1_PWREN			(1 << 28)
#define RCC_APBENR1_DBGEN			(1 << 27)
#define RCC_APBENR1_I2C2EN			(1 << 22)
#define RCC_APBENR1_I2C1EN			(1 << 21)
#define RCC_APBENR1_USART4EN		(1 << 19)
#define RCC_APBENR1_USART3EN		(1 << 18)
#define RCC_APBENR1_USART2EN		(1 << 17)
#define RCC_APBENR1_SPI2EN			(1 << 14)
#define RCC_APBENR1_WWDGEN			(1 << 11)
#define RCC_APBENR1_RTCAPBEN		(1 << 10)
#define RCC_APBENR1_TIM3EN			(1 << 1)
#define RCC_APBENR1_TIM2EN			(1 << 0)
/**@}*/

/** @defgroup rcc_apbenr2_en RCC_APBENR2 enable values
@{*/
#define RCC_APBENR2_ADCEN			(1 << 20)
#define RCC_APBENR2_TIM17EN			(1 << 18)
#define RCC_APBENR2_TIM16EN			(1 << 17)
#define RCC_APBENR2_TIM16EN			(1 << 17)
#define RCC_APBENR2_TIM15EN			(1 << 16)
#define RCC_APBENR2_TIM14EN			(1 << 15)
#define RCC_APBENR2_USART1EN		(1 << 14)
#define RCC_APBENR2_SPI1EN			(1 << 12)
#define RCC_APBENR2_TIM1EN			(1 << 11)
#define RCC_APBENR2_SYSCFGEN		(1 << 0)
/**@}*/
/**@}*/

/** @defgroup rcc_aphbsmenr_en RCC_AHBSMENR enable in sleep/stop mode values
@{*/
#define RCC_AHBSMENR_CRCSMEN			(1 << 12)
#define RCC_AHBSMENR_SRAMSMEN			(1 << 9)
#define RCC_AHBSMENR_FLASHSMEN			(1 << 8)
#define RCC_AHBSMENR_DMASMEN			(1 << 0)
/**@}*/

/** @defgroup rcc_apbsmenr_en RCC_APBSMENR1 enable in sleep/stop mode values
@{*/
#define RCC_APBSMENR1_PWRSMEN			(1 << 28)
#define RCC_APBSMENR1_DBGSMEN			(1 << 27)
#define RCC_APBSMENR1_I2C2SMEN			(1 << 22)
#define RCC_APBSMENR1_I2C1SMEN			(1 << 21)
#define RCC_APBSMENR1_USART4SMEN		(1 << 19)
#define RCC_APBSMENR1_USART3SMEN		(1 << 18)
#define RCC_APBSMENR1_USART2SMEN		(1 << 17)
#define RCC_APBSMENR1_SPI2SMEN			(1 << 14)
#define RCC_APBSMENR1_WWDGSMEN			(1 << 11)
#define RCC_APBSMENR1_RTCAPBSMEN			(1 << 10)
#define RCC_APBSMENR1_TIM3SMEN			(1 << 1)
#define RCC_APBSMENR1_TIM2SMEN			(1 << 0)
/**@}*/

/** @defgroup rcc_apbsmenr2_en RCC_APBSMENR2 enable in sleep/stop mode values
@{*/
#define RCC_APBSMENR2_ADCSMEN			(1 << 20)
#define RCC_APBSMENR2_TIM17SMEN			(1 << 18)
#define RCC_APBSMENR2_TIM16SMEN			(1 << 17)
#define RCC_APBSMENR2_TIM15SMEN			(1 << 16)
#define RCC_APBSMENR2_TIM14SMEN			(1 << 15)
#define RCC_APBSMENR2_USART1SMEN		(1 << 14)
#define RCC_APBSMENR2_SPI1SMEN			(1 << 12)
#define RCC_APBSMENR2_TIM1SMEN			(1 << 11)
#define RCC_APBSMENR2_SYSCFGSMEN		(1 << 0)
/**@}*/


/** @defgroup rcc_ccipr CCIPR Peripherals Independent Clock Config Register
@{*/
#define RCC_CCIPR_ADCSEL_MASK		0x3
#define RCC_CCIPR_ADCSEL_SHIFT		30
/** @defgroup rcc_ccipr_adcsel ADCSEL
@{*/
#define RCC_CCIPR_ADCSEL_SYSCLK		0
#define RCC_CCIPR_ADCSEL_HSIKER		2
/**@}*/

#define RCC_CCIPR_I2S1SEL_MASK		0x3
#define RCC_CCIPR_I2S1SEL_SHIFT		14
/** @defgroup rcc_ccipr_i2s1sel I2S1SEL I2S1 Clock source selection
@{*/
#define RCC_CCIPR_I2S1SEL_SYSCLK		0
#define RCC_CCIPR_I2S1SEL_HSIKER		2
#define RCC_CCIPR_I2S1SEL_I2S_CKIN		3
/**@}*/

#define RCC_CCIPR_I2CxSEL_MASK		0x3
#define RCC_CCIPR_I2C1SEL_SHIFT		12
#define RCC_CCIPR_I2C2SEL_SHIFT		14
/** @defgroup rcc_ccipr_i2c1sel I2C1SEL I2C1 Clock source selection
@{*/
#define RCC_CCIPR_I2CxSEL_PCLK			0
#define RCC_CCIPR_I2CxSEL_SYSCLK		1
#define RCC_CCIPR_I2CxSEL_HSIKER		2
/**@}*/

#define RCC_CCIPR_USARTxSEL_MASK		0x3
#define RCC_CCIPR_USART1SEL_SHIFT		0
/** @defgroup rcc_ccipr_usartxsel USARTxSEL USARTx Clock source selection
@{*/
#define RCC_CCIPR_USARTxSEL_PCLK		0
#define RCC_CCIPR_USARTxSEL_SYSCLK		1
#define RCC_CCIPR_USARTxSEL_HSIKER		2
#define RCC_CCIPR_USARTxSEL_LSE			3
/**@}*/

/**@}*/

/** @defgroup rcc_ccipr2 CCIPR2 Peripherals Independent Clock Config Register 2
@{*/
#define RCC_CCIPR2_USBSEL			(1 << 12)
/**@}*/

/**@}*/

/** @defgroup rcc_csr1 CSR1 Control and Status Register 1
@{*/
#define RCC_CSR1_LSCOSEL			(1 << 25)
#define RCC_CSR1_LSCOEN				(1 << 24)
#define RCC_CSR1_RTCRST				(1 << 16)
#define RCC_CSR1_RTCEN				(1 << 15)

#define RCC_CSR1_RTCSEL_SHIFT			8
#define RCC_CSR1_RTCSEL_MASK			3

#define RCC_CSR1_RTCSEL_NONE			0
#define RCC_CSR1_RTCSEL_LSE				1
#define RCC_CSR1_RTCSEL_LSI				2
#define RCC_CSR1_RTCSEL_HSEDIV32		3

#define RCC_CSR1_LSECSSD			(1 << 6)
#define RCC_CSR1_LSECSSON			(1 << 5)
#define RCC_CSR1_LSEDRV				(1 << 3)
#define RCC_CSR1_LSEBYP				(1 << 2)
#define RCC_CSR1_LSERDY				(1 << 1)
#define RCC_CSR1_LSEON				(1 << 0)
/**@}*/

/* needed for compatibility */
#define RCC_CSR				RCC_CSR1
#define RCC_CSR_LSEBYP		RCC_CSR1_LSEBYP

/** @defgroup rcc_csr2 CSR2 Control and Status Register 2
@{*/
#define RCC_CSR2_LPWRRSTF			(1 << 31)
#define RCC_CSR2_WWDGRSTF			(1 << 30)
#define RCC_CSR2_IWDGRSTF			(1 << 29)
#define RCC_CSR2_SFTRSTF			(1 << 28)
#define RCC_CSR2_PWRRSTF			(1 << 27)
#define RCC_CSR2_PINRSTF			(1 << 26)
#define RCC_CSR2_OBLRSTF			(1 << 25)
#define RCC_CSR2_RMVF				(1 << 23)
#define RCC_CSR2_LSIRDY				(1 << 1)
#define RCC_CSR2_LSION				(1 << 0)
/**@}*/

/* --- Variable definitions ------------------------------------------------ */

extern uint32_t rcc_ahb_frequency;
extern uint32_t rcc_apb1_frequency;
/*
 * as done for F0, fake out apb2_frequency as the device does not really have
 * apb2 clock.
 */
#define rcc_apb2_frequency rcc_apb1_frequency

/* --- Function prototypes ------------------------------------------------- */

#define _REG_BIT(offset, bit)            (((offset) << 5) + (bit))

enum rcc_osc {
	RCC_HSI48,
	RCC_HSIUSB48,
	RCC_HSE,
	RCC_LSE,
	RCC_LSI,
};

enum rcc_periph_clken {
	RCC_GPIOF = _REG_BIT(RCC_IOPENR_OFFSET, 5),
	RCC_GPIOE = _REG_BIT(RCC_IOPENR_OFFSET, 4),
	RCC_GPIOD = _REG_BIT(RCC_IOPENR_OFFSET, 3),
	RCC_GPIOC = _REG_BIT(RCC_IOPENR_OFFSET, 2),
	RCC_GPIOB = _REG_BIT(RCC_IOPENR_OFFSET, 1),
	RCC_GPIOA = _REG_BIT(RCC_IOPENR_OFFSET, 0),

	RCC_CRC = _REG_BIT(RCC_AHBENR_OFFSET, 12),
	RCC_FLASH = _REG_BIT(RCC_AHBENR_OFFSET, 8),
	RCC_DMA1 = _REG_BIT(RCC_AHBENR_OFFSET, 0),
	RCC_DMA = _REG_BIT(RCC_AHBENR_OFFSET, 0), /* Compatibility */

	RCC_PWR = _REG_BIT(RCC_APBENR1_OFFSET, 28),
	RCC_DBG = _REG_BIT(RCC_APBENR1_OFFSET, 27),
	RCC_I2C2 = _REG_BIT(RCC_APBENR1_OFFSET, 22),
	RCC_I2C1 = _REG_BIT(RCC_APBENR1_OFFSET, 21),
	RCC_USART4 = _REG_BIT(RCC_APBENR1_OFFSET, 19),
	RCC_USART3 = _REG_BIT(RCC_APBENR1_OFFSET, 18),
	RCC_USART2 = _REG_BIT(RCC_APBENR1_OFFSET, 17),
	RCC_CRS = _REG_BIT(RCC_APBENR1_OFFSET, 16),
	RCC_SPI2 = _REG_BIT(RCC_APBENR1_OFFSET, 14),
	RCC_USB = _REG_BIT(RCC_APBENR1_OFFSET, 13),
	RCC_FDCAN = _REG_BIT(RCC_APBENR1_OFFSET, 12),
	RCC_WWDG = _REG_BIT(RCC_APBENR1_OFFSET, 11),
	RCC_RTCAPB = _REG_BIT(RCC_APBENR1_OFFSET, 10),
	RCC_TIM3 = _REG_BIT(RCC_APBENR1_OFFSET, 1),
	RCC_TIM2 = _REG_BIT(RCC_APBENR1_OFFSET, 0),

	RCC_ADC = _REG_BIT(RCC_APBENR2_OFFSET, 20),
	RCC_TIM17 = _REG_BIT(RCC_APBENR2_OFFSET, 18),
	RCC_TIM16 = _REG_BIT(RCC_APBENR2_OFFSET, 17),
	RCC_TIM15 = _REG_BIT(RCC_APBENR2_OFFSET, 16),
	RCC_TIM14 = _REG_BIT(RCC_APBENR2_OFFSET, 15),
	RCC_USART1 = _REG_BIT(RCC_APBENR2_OFFSET, 14),
	RCC_SPI1 = _REG_BIT(RCC_APBENR2_OFFSET, 12),
	RCC_TIM1 = _REG_BIT(RCC_APBENR2_OFFSET, 11),
	RCC_SYSCFG = _REG_BIT(RCC_APBENR2_OFFSET, 0),

	SCC_GPIOF = _REG_BIT(RCC_IOPSMENR_OFFSET, 5),
	SCC_GPIOE = _REG_BIT(RCC_IOPSMENR_OFFSET, 4),
	SCC_GPIOD = _REG_BIT(RCC_IOPSMENR_OFFSET, 3),
	SCC_GPIOC = _REG_BIT(RCC_IOPSMENR_OFFSET, 2),
	SCC_GPIOB = _REG_BIT(RCC_IOPSMENR_OFFSET, 1),
	SCC_GPIOA = _REG_BIT(RCC_IOPSMENR_OFFSET, 0),

	SCC_CRC = _REG_BIT(RCC_AHBSMENR_OFFSET, 12),
	SCC_SRAM = _REG_BIT(RCC_AHBSMENR_OFFSET, 9),
	SCC_FLASH = _REG_BIT(RCC_AHBSMENR_OFFSET, 8),
	SCC_DMA1 = _REG_BIT(RCC_AHBSMENR_OFFSET, 0),
	SCC_DMA = _REG_BIT(RCC_AHBSMENR_OFFSET, 0), /* Compatibility */

	SCC_PWR = _REG_BIT(RCC_APBSMENR1_OFFSET, 28),
	SCC_DBG = _REG_BIT(RCC_APBSMENR1_OFFSET, 27),
	SCC_I2C2 = _REG_BIT(RCC_APBSMENR1_OFFSET, 22),
	SCC_I2C1 = _REG_BIT(RCC_APBSMENR1_OFFSET, 21),
	SCC_USART4 = _REG_BIT(RCC_APBSMENR1_OFFSET, 19),
	SCC_USART3 = _REG_BIT(RCC_APBSMENR1_OFFSET, 18),
	SCC_USART2 = _REG_BIT(RCC_APBSMENR1_OFFSET, 17),
	SCC_CRS = _REG_BIT(RCC_APBSMENR1_OFFSET, 16),
	SCC_SPI2 = _REG_BIT(RCC_APBSMENR1_OFFSET, 14),
	SCC_USB = _REG_BIT(RCC_APBSMENR1_OFFSET, 13),
	SCC_FDCAN = _REG_BIT(RCC_APBSMENR1_OFFSET, 12),
	SCC_WWDG = _REG_BIT(RCC_APBSMENR1_OFFSET, 11),
	SCC_RTCAPB = _REG_BIT(RCC_APBSMENR1_OFFSET, 10),
	SCC_TIM3 = _REG_BIT(RCC_APBSMENR1_OFFSET, 1),
	SCC_TIM2 = _REG_BIT(RCC_APBSMENR1_OFFSET, 0),

	SCC_ADC = _REG_BIT(RCC_APBSMENR2_OFFSET, 20),
	SCC_TIM17 = _REG_BIT(RCC_APBSMENR2_OFFSET, 18),
	SCC_TIM16 = _REG_BIT(RCC_APBSMENR2_OFFSET, 17),
	SCC_TIM15 = _REG_BIT(RCC_APBSMENR2_OFFSET, 16),
	SCC_TIM14 = _REG_BIT(RCC_APBSMENR2_OFFSET, 15),
	SCC_USART1 = _REG_BIT(RCC_APBSMENR2_OFFSET, 14),
	SCC_SPI1 = _REG_BIT(RCC_APBSMENR2_OFFSET, 12),
	SCC_TIM1 = _REG_BIT(RCC_APBSMENR2_OFFSET, 11),
	SCC_SYSCFG = _REG_BIT(RCC_APBSMENR2_OFFSET, 0),
};

enum rcc_periph_rst {
	RST_GPIOF = _REG_BIT(RCC_IOPRSTR_OFFSET, 5),
	RST_GPIOE = _REG_BIT(RCC_IOPRSTR_OFFSET, 4),
	RST_GPIOD = _REG_BIT(RCC_IOPRSTR_OFFSET, 3),
	RST_GPIOC = _REG_BIT(RCC_IOPRSTR_OFFSET, 2),
	RST_GPIOB = _REG_BIT(RCC_IOPRSTR_OFFSET, 1),
	RST_GPIOA = _REG_BIT(RCC_IOPRSTR_OFFSET, 0),

	RST_CRC = _REG_BIT(RCC_AHBRSTR_OFFSET, 12),
	RST_FLASH = _REG_BIT(RCC_AHBRSTR_OFFSET, 8),
	RST_DMA1 = _REG_BIT(RCC_AHBRSTR_OFFSET, 0),
	RST_DMA = _REG_BIT(RCC_AHBRSTR_OFFSET, 0), /* Compatibility */

	RST_PWR = _REG_BIT(RCC_APBRSTR1_OFFSET, 28),
	RST_DBG = _REG_BIT(RCC_APBRSTR1_OFFSET, 27),
	RST_I2C2 = _REG_BIT(RCC_APBRSTR1_OFFSET, 22),
	RST_I2C1 = _REG_BIT(RCC_APBRSTR1_OFFSET, 21),
	RST_USART4 = _REG_BIT(RCC_APBRSTR1_OFFSET, 19),
	RST_USART3 = _REG_BIT(RCC_APBRSTR1_OFFSET, 18),
	RST_USART2 = _REG_BIT(RCC_APBRSTR1_OFFSET, 17),
	RST_CRS = _REG_BIT(RCC_APBRSTR1_OFFSET, 16),
	RST_SPI2 = _REG_BIT(RCC_APBRSTR1_OFFSET, 14),
	RST_USB = _REG_BIT(RCC_APBRSTR1_OFFSET, 13),
	RST_FDCAN = _REG_BIT(RCC_APBRSTR1_OFFSET, 12),
	RST_TIM3 = _REG_BIT(RCC_APBRSTR1_OFFSET, 1),
	RST_TIM2 = _REG_BIT(RCC_APBRSTR1_OFFSET, 0),

	RST_ADC = _REG_BIT(RCC_APBRSTR2_OFFSET, 20),
	RST_TIM17 = _REG_BIT(RCC_APBRSTR2_OFFSET, 18),
	RST_TIM16 = _REG_BIT(RCC_APBRSTR2_OFFSET, 17),
	RST_TIM15 = _REG_BIT(RCC_APBRSTR2_OFFSET, 16),
	RST_TIM14 = _REG_BIT(RCC_APBRSTR2_OFFSET, 15),
	RST_USART1 = _REG_BIT(RCC_APBRSTR2_OFFSET, 14),
	RST_SPI1 = _REG_BIT(RCC_APBRSTR2_OFFSET, 12),
	RST_TIM1 = _REG_BIT(RCC_APBRSTR2_OFFSET, 11),
	RST_SYSCFG = _REG_BIT(RCC_APBRSTR2_OFFSET, 0),
};

struct rcc_clock_scale {
	enum rcc_osc sysclock_source;

	uint8_t hsisys_div;

	uint8_t hpre;
	uint8_t ppre;
	uint8_t flash_waitstates;

	uint32_t ahb_frequency;
	uint32_t apb_frequency;
};

enum rcc_clock {
	RCC_CLOCK_CONFIG_HSI_48MHZ,
	RCC_CLOCK_CONFIG_END
};

extern const struct rcc_clock_scale rcc_clock_config[RCC_CLOCK_CONFIG_END];

#include <libopencm3/stm32/common/rcc_common_all.h>

BEGIN_DECLS

void rcc_osc_on(enum rcc_osc osc);
void rcc_osc_off(enum rcc_osc osc);

void rcc_css_enable(void);
void rcc_css_disable(void);
void rcc_css_int_clear(void);
int rcc_css_int_flag(void);

void rcc_set_sysclk_source(enum rcc_osc osc);
enum rcc_osc rcc_get_usbclk_source(void);
void rcc_set_usbclk_source(enum rcc_osc clk);
void rcc_wait_for_sysclk_status(enum rcc_osc osc);
enum rcc_osc rcc_system_clock_source(void);

void rcc_set_ppre(uint32_t ppre);
void rcc_set_hpre(uint32_t hpre);
void rcc_set_hsisys_div(uint32_t sysdiv);
void rcc_set_mcopre(uint32_t mcopre);

void rcc_clock_setup(const struct rcc_clock_scale *clock);

void rcc_set_peripheral_clk_sel(uint32_t periph, uint32_t sel);
uint32_t rcc_get_usart_clk_freq(uint32_t usart);
uint32_t rcc_get_timer_clk_freq(uint32_t timer);
uint32_t rcc_get_i2c_clk_freq(uint32_t i2c);
uint32_t rcc_get_spi_clk_freq(uint32_t spi);

END_DECLS

/**@}*/

#endif

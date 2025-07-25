#if defined(STM32F0)
#	include <libopencmsis/stm32/f0/irqhandlers.h>
#elif defined(STM32F1)
#	include <libopencmsis/stm32/f1/irqhandlers.h>
#elif defined(STM32F2)
#	include <libopencmsis/stm32/f2/irqhandlers.h>
#elif defined(STM32F3)
#	include <libopencmsis/stm32/f3/irqhandlers.h>
#elif defined(STM32F4)
#	include <libopencmsis/stm32/f4/irqhandlers.h>
#elif defined(STM32F7)
#	include <libopencmsis/stm32/f7/irqhandlers.h>
#elif defined(STM32L0)
#	include <libopencmsis/stm32/l0/irqhandlers.h>
#elif defined(STM32L1)
#	include <libopencmsis/stm32/l1/irqhandlers.h>
#elif defined(STM32L4)
#	include <libopencmsis/stm32/l4/irqhandlers.h>
#elif defined(STM32U5)
#	include <libopencmsis/stm32/u5/irqhandlers.h>
#elif defined(STM32G0)
#	include <libopencmsis/stm32/g0/irqhandlers.h>
#elif defined(STM32C0)
#	include <libopencmsis/stm32/c0/irqhandlers.h>
#elif defined(STM32G4)
#	include <libopencmsis/stm32/g4/irqhandlers.h>
#elif defined(STM32H7)
#	include <libopencmsis/stm32/h7/irqhandlers.h>

#elif defined(GD32F1X0)
#	include <libopencmsis/gd32/f1x0/irqhandlers.h>

#elif defined(EFM32TG)
#	include <libopencmsis/efm32/efm32tg/irqhandlers.h>
#elif defined(EFM32G)
#	include <libopencmsis/efm32/efm32g/irqhandlers.h>
#elif defined(EFM32HG)
#	include <libopencmsis/efm32/efm32hg/irqhandlers.h>
#elif defined(EFM32LG)
#	include <libopencmsis/efm32/efm32lg/irqhandlers.h>
#elif defined(EFM32GG)
#	include <libopencmsis/efm32/efm32gg/irqhandlers.h>

#elif defined(LPC13XX)
#       include <libopencmsis/lpc13xx/irqhandlers.h>
#elif defined(LPC17XX)
#       include <libopencmsis/lpc17xx/irqhandlers.h>
#elif defined(LPC43XX_M4)
#       include <libopencmsis/lpc43xx/m4/irqhandlers.h>
#elif defined(LPC43XX_M0)
#       include <libopencmsis/lpc43xx/m0/irqhandlers.h>

#elif defined(SAM3A)
#       include <libopencmsis/sam/3a/irqhandlers.h>
#elif defined(SAM3N)
#       include <libopencmsis/sam/3n/irqhandlers.h>
#elif defined(SAM3S)
#       include <libopencmsis/sam/3s/irqhandlers.h>
#elif defined(SAM3U)
#       include <libopencmsis/sam/3u/irqhandlers.h>
#elif defined(SAM3X)
#       include <libopencmsis/sam/3x/irqhandlers.h>
#elif defined(SAMD)
#       include <libopencmsis/sam/d/irqhandlers.h>

#elif defined(LM3S) || defined(LM4F)
/* Yes, we use the same interrupt table for both LM3S and LM4F */
#	include <libopencmsis/lm3s/irqhandlers.h>

#elif defined(SWM050)
#	include <libopencmsis/swm050/irqhandlers.h>
#else
#	warning"no chipset defined; user interrupts are not redirected"

#endif

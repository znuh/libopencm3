/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2014 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
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

/* Note: could check for (un)aligned access being prohibited with defined(__ARM_ARCH_6M__) */

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/rcc.h>

/* check if Clock Recovery System is available */
#ifdef CRS_BASE
#include <libopencm3/stm32/crs.h>
#endif

#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include "../usb/usb_private.h"
#include "common/st_usbfs_core.h"

static struct _usbd_device st_usbfs_dev;
static uint16_t epbuf_addr[USB_MAX_ENDPOINTS][2];

/* --- Dedicated packet buffer memory SRAM access scheme: 32 bits ---------------------- */
#ifdef ST_USBFS_PMA_AS_1X32

static inline uint16_t assign_buffer_1x32(uint16_t ep_id, uint32_t dir_tx, uint16_t *ram_ofs, uint16_t rx_blocks) {
	uint16_t ofs = (*ram_ofs + 3) & ~3;
	*ram_ofs = ofs;
	if(!dir_tx)
		*USB_CHEP_RXTXBD(ep_id) = (rx_blocks << CHEP_BD_COUNT_SHIFT) | ofs;
	return ofs;
}

/* NOTE: could check if src buf is 32-Bit aligned (!(buf&3)) and do a faster copy then */
static inline void copy_to_pm_1x32(uint16_t ep_id, uint16_t txbuf_ofs, const void *vsrc, uint16_t len)
{
	volatile uint32_t *PM = (volatile uint32_t *) (USB_PMA_BASE + txbuf_ofs);
	const uint8_t *src = vsrc;
	uint32_t i,v;

	for (v=i=0; i<len; i++, src++) {
		v<<=8;
		v|=*src;
		if((i&3) == 3)
			*PM++ = __builtin_bswap32(v);
	}

	// remainder?
	i = i&3;
	if(i) {
		i = (4-i) << 3;
		*PM = __builtin_bswap32(v<<i);
	}

	*USB_CHEP_TXRXBD(ep_id) = (len << CHEP_BD_COUNT_SHIFT) | txbuf_ofs;
}

/* ERRATUM for STM32C071x8/xB (ES0618 - Rev 1):
 *  Buffer description table update completes after CTR interrupt triggers
 * Description:
 *  During OUT transfers, the correct transfer interrupt (CTR) is triggered a little before the last USB SRAM accesses
 *  have completed. If the software responds quickly to the interrupt, the full buffer contents may not be correct.
 * Workaround:
 *  Software should ensure that a small delay is included before accessing the SRAM contents. This delay should be
 *  800 ns in Full Speed mode and 6.4 μs in Low Speed mode.
 * ---------------------------------------------------------------------------------------------------------------------
 * At 48MHz the delay needed is 39 cycles.
 * Counting instruction cycles in the disassembly from st_usbfs_poll: istr = *USB_ISTR_REG
 * to this function entry takes >50 cycles, so no delay needed here for Full Speed.
 * Will fail for Low Speed mode, but LS mode is too exotic to justify a synthetic delay here.
 */
/* NOTE: could check if dst buf is 32-Bit aligned (!(buf&3)) and do a faster copy then */
static inline uint16_t copy_from_pm_1x32(uint16_t ep_id, uint16_t rxbuf_ofs, void *vdst, uint16_t len)
{
	uint32_t v, i, count = (*USB_CHEP_RXTXBD(ep_id) >> CHEP_BD_COUNT_SHIFT) & CHEP_BD_COUNT_MASK;
	volatile uint32_t *PM = (volatile uint32_t *) (USB_PMA_BASE + rxbuf_ofs);
	uint8_t *dst = vdst;

	count = MIN(count, len);
	for(v=i=0; i<count; i++, dst++, v>>=8) {
		if(!(i&3))
			v = *PM++;
		*dst = v&0xff;
	}
	return count;
}

#endif /* ST_USBFS_PMA_AS_1X32 */

/* --- Dedicated packet buffer memory SRAM access scheme: 2 x 16 bits / word ------------- */
#ifdef ST_USBFS_PMA_AS_2X16

static inline uint16_t assign_buffer_2x16(uint16_t ep_id, uint32_t dir_tx, uint16_t *ram_ofs, uint16_t rx_blocks) {
	uint16_t ofs = *ram_ofs;
	USB_BT16_SET(dir_tx ? BT_TX_ADDR(ep_id) : BT_RX_ADDR(ep_id), ofs);
	if(!dir_tx)
		USB_BT16_SET(BT_RX_COUNT(ep_id), rx_blocks);
	return ofs;
}

static inline void copy_to_pm_2x16(uint16_t ep_id, uint16_t txbuf_ofs, const void *vsrc, uint16_t len)
{
	volatile uint16_t *PM = (volatile void *)(USB_PMA_BASE + txbuf_ofs);
	const uint8_t *src = vsrc;
	uint32_t n_words = len >> 1;

	/* This is a bytewise copy, so it always works, even on CM0(+)
	 * that don't support unaligned accesses. */

	/* copy complete words */
	for(;n_words;n_words--,src+=2)
		*PM++ = (uint16_t)src[1] << 8 | *src;

	/* copy remaining byte if odd length */
	if(len&1)
		*PM = *src;

	USB_BT16_SET(BT_TX_COUNT(ep_id), len);
}

static inline uint16_t copy_from_pm_2x16(uint16_t ep_id, uint16_t rxbuf_ofs, void *dst, uint16_t len)
{
	const volatile uint16_t *PM = (volatile void *)(USB_PMA_BASE + rxbuf_ofs);
	uint16_t res = MIN(USB_BT16_GET(BT_RX_COUNT(ep_id)) & 0x3ff, len);
	uint8_t odd = res & 1;
	len = res >> 1;

	if (((uintptr_t) dst) & 0x01) {
		for (; len; PM++, len--) {
			uint16_t value = *PM;
			*(uint8_t *) dst++ = value;
			*(uint8_t *) dst++ = value >> 8;
		}
	} else {
		for (; len; PM++, dst += 2, len--) {
			*(uint16_t *) dst = *PM;
		}
	}

	if (odd) {
		*(uint8_t *) dst = *(uint8_t *) PM;
	}
	return res;
}

#endif /* ST_USBFS_PMA_AS_2X16 */

/* --- Dedicated packet buffer memory SRAM access scheme: 1 x 16 bits / word -------- */
#ifdef ST_USBFS_PMA_AS_1X16

static inline uint16_t assign_buffer_1x16(uint16_t ep_id, uint32_t dir_tx, uint16_t *ram_ofs, uint16_t rx_blocks) {
	uint16_t ofs = *ram_ofs;
	USB_BT32_SET(dir_tx ? BT_TX_ADDR(ep_id) : BT_RX_ADDR(ep_id), ofs);
	if(!dir_tx)
		USB_BT32_SET(BT_RX_COUNT(ep_id), rx_blocks);
	return ofs << 1; /* PMA access addr. must be multiplied by 2 */
}

static inline void copy_to_pm_1x16(uint16_t ep_id, uint16_t txbuf_ofs, const void *vsrc, uint16_t len)
{
	volatile uint32_t *PM = (volatile void *)(USB_PMA_BASE + txbuf_ofs);
	uint32_t n_words = len >> 1;
	const uint16_t *src = vsrc;

	/* copy complete words */
	for(;n_words;n_words--)
		*PM++ = *src++;

	/* copy remaining byte if odd length */
	if(len&1)
		*PM = *((const uint8_t *)src);

	USB_BT32_SET(BT_TX_COUNT(ep_id), len);
}

static inline uint16_t copy_from_pm_1x16(uint16_t ep_id, uint16_t rxbuf_ofs, void *vdst, uint16_t len)
{
	const volatile uint16_t *PM = (volatile void *)(USB_PMA_BASE + rxbuf_ofs);
	uint16_t res = MIN(USB_BT32_GET(BT_RX_COUNT(ep_id)) & 0x3ff, len);
	uint16_t *dst = vdst;
	uint8_t odd = res & 1;
	len = res;

	for (len >>= 1; len; PM += 2, dst++, len--)
		*dst = *PM;
	if (odd)
		*(uint8_t *) dst = *(uint8_t *) PM;
	return res;
}

#endif /* ST_USBFS_PMA_AS_1X16 */

/* --- Common (dispatch) functions for all peripheral variants ---------------------- */

/**
 * Assign a data buffer in packet memory for an endpoint
 *
 * @param ep_id Endpoint ID (0..7)
 * @param dir_tx 1 if TX endpoint, 0 for RX
 * @param ram_ofs Pointer to RAM offset for packet buffer
 * @param rx_blocks BLSIZE / NUM_BLOCK[4:0] (shifted) for rxcount register - 0 for TX
 */
void st_usbfs_assign_buffer(uint16_t ep_id, uint32_t dir_tx, uint16_t *ram_ofs, uint16_t rx_blocks)
{
	uint16_t ofs;
#if defined(ST_USBFS_PMA_AS_1X32)
	ofs = assign_buffer_1x32(ep_id, dir_tx, ram_ofs, rx_blocks);
#elif defined(ST_USBFS_PMA_AS_2X16)
	ofs = assign_buffer_2x16(ep_id, dir_tx, ram_ofs, rx_blocks);
#elif defined(ST_USBFS_PMA_AS_1X16)
	ofs = assign_buffer_1x16(ep_id, dir_tx, ram_ofs, rx_blocks);
#else
#error "unknown PMA access scheme"
#endif
	epbuf_addr[ep_id][dir_tx] = ofs;
}

void st_usbfs_copy_to_pm(uint16_t ep_id, const void *src, uint16_t len)
{
	uint16_t txbuf_ofs = epbuf_addr[ep_id][USB_BUF_TX];

#if defined(ST_USBFS_PMA_AS_1X32)
	copy_to_pm_1x32(ep_id, txbuf_ofs, src, len);
#elif defined(ST_USBFS_PMA_AS_2X16)
	copy_to_pm_2x16(ep_id, txbuf_ofs, src, len);
#elif defined(ST_USBFS_PMA_AS_1X16)
	copy_to_pm_1x16(ep_id, txbuf_ofs, src, len);
#else
#error "unknown PMA access scheme"
#endif
}

/**
 * Copy a data buffer from packet memory.
 *
 * @param ep_id Endpoint ID (0..7)
 * @param buf Destination pointer for data buffer.
 * @param len Number of bytes to copy.
 */
uint16_t st_usbfs_copy_from_pm(uint16_t ep_id, void *dst, uint16_t len)
{
	uint16_t rxbuf_ofs = epbuf_addr[ep_id][USB_BUF_RX];

#if defined(ST_USBFS_PMA_AS_1X32)
	return copy_from_pm_1x32(ep_id, rxbuf_ofs, dst, len);
#elif defined(ST_USBFS_PMA_AS_2X16)
	return copy_from_pm_2x16(ep_id, rxbuf_ofs, dst, len);
#elif defined(ST_USBFS_PMA_AS_1X16)
	return copy_from_pm_1x16(ep_id, rxbuf_ofs, dst, len);
#else
#error "unknown PMA access scheme"
#endif
}

#ifdef ST_USBFS_HAVE_BCD
static void st_usbfs_disconnect(usbd_device *usbd_dev, bool disconnected)
{
	(void)usbd_dev;
	uint16_t reg = GET_REG(USB_BCDR_REG);
	SET_REG(USB_BCDR_REG, disconnected ? (reg & ~USB_BCDR_DPPU) : (reg | USB_BCDR_DPPU));
}
#endif

/** Initialize the USB device controller hardware of the STM32. */
static usbd_device *st_usbfs_usbd_init(void)
{
#if defined(STM32C0)
	/* make things easier for user by handling sane defaults */
	if(rcc_get_usbclk_source() == RCC_HSIUSB48) {
		rcc_osc_on(RCC_HSIUSB48);
		rcc_wait_for_osc_ready(RCC_HSIUSB48);
		crs_autotrim_usb_enable();
	}
#endif

	rcc_periph_clock_enable(RCC_USB);

	/* Reset might be cleared and endpoints might be active, if a
	 * bootloader used USB before. Therefore, enforce a reset here */
	SET_REG(USB_CNTR_REG, USB_CNTR_FRES);

	/* We need to keep reset enabled for t_STARTUP after
	 * clearing powerdown or the transceiver won't work yet.
	 * Experiments showed that not doing this will cause the
	 * USB interface to enter error mode (USB_ISTR_ERR set).
	 * Datasheet states t_STARTUP: 1us */
	for(uint32_t i=128;i;i--) __asm__("nop");

	SET_REG(USB_CNTR_REG, 0);

#ifdef ST_USBFS_HAVE_BTADDR
	SET_REG(USB_BTABLE_REG, USB_BTABLE_OFS);
#endif

	SET_REG(USB_ISTR_REG, 0);

	/* Enable RESET, SUSPEND, RESUME and CTR interrupts. */
	SET_REG(USB_CNTR_REG, USB_CNTR_RESETM | USB_CNTR_CTRM |
		USB_CNTR_SUSPM | USB_CNTR_WKUPM);

#ifdef ST_USBFS_HAVE_BCD
	/* enable pullup */
	SET_REG(USB_BCDR_REG, USB_BCDR_DPPU);
#endif

	return &st_usbfs_dev;
}

const struct _usbd_driver st_usbfs_usb_driver = {
	.init = st_usbfs_usbd_init,
	.set_address = st_usbfs_set_address,
	.ep_setup = st_usbfs_ep_setup,
	.ep_reset = st_usbfs_endpoints_reset,
	.ep_stall_set = st_usbfs_ep_stall_set,
	.ep_stall_get = st_usbfs_ep_stall_get,
	.ep_nak_set = st_usbfs_ep_nak_set,
	.ep_write_packet = st_usbfs_ep_write_packet,
	.ep_read_packet = st_usbfs_ep_read_packet,
#ifdef ST_USBFS_HAVE_BCD
	.disconnect = st_usbfs_disconnect,
#endif
	.poll = st_usbfs_poll,
};

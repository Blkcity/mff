/****************************************************************************
 *
 *   Copyright (c) 2013 FMX10 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name FMX10 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fmx10fmu2_led.c
 *
 * FMX10FMU LED backend.
 */

#include <fmx10_config.h>

#include <stdbool.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

#ifdef CONFIG_ARCH_LEDS
static bool nuttx_owns_leds = true;
//                                B  R  S  G
//                                0  1  2  3
static const uint8_t xlatfmx10[] = {1, 2, 4, 0};
#  define xlat(p) xlatfmx10[(p)]
static uint32_t g_ledmap[] = {
	GPIO_LED_GREEN,   // Indexed by BOARD_LED_GREEN
	GPIO_LED_BLUE,    // Indexed by BOARD_LED_BLUE
	GPIO_LED_RED,     // Indexed by BOARD_LED_RED
	GPIO_LED_SAFETY,  // Indexed by LED_SAFETY by xlatfmx10
};

#else

#  define xlat(p)
static uint32_t g_ledmap[] = {
	GPIO_LED_BLUE,    // Indexed by LED_BLUE
	GPIO_LED_RED,     // Indexed by LED_RED, LED_AMBER
	GPIO_LED_SAFETY,  // Indexed by LED_SAFETY
	GPIO_LED_GREEN,   // Indexed by LED_GREEN
};

#endif

__EXPORT void led_init(void)
{
	/* Configure LED GPIOs for output */
	for (size_t l = 0; l < (sizeof(g_ledmap) / sizeof(g_ledmap[0])); l++) {
		stm32_configgpio(g_ledmap[l]);
	}
}

static void phy_set_led(int led, bool state)
{
	/* Drive High to switch on */

	stm32_gpiowrite(g_ledmap[led], state);
}

static bool phy_get_led(int led)
{

	return stm32_gpioread(g_ledmap[led]);
}

__EXPORT void led_on(int led)
{
	phy_set_led(xlat(led), true);
}

__EXPORT void led_off(int led)
{
	phy_set_led(xlat(led), false);
}

__EXPORT void led_toggle(int led)
{

	phy_set_led(xlat(led), !phy_get_led(xlat(led)));
}

#ifdef CONFIG_ARCH_LEDS
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
	led_init();
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
	if (!nuttx_owns_leds) {
		return;
	}

	switch (led) {
	default:
		break;

	case LED_HEAPALLOCATE:
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_IRQSENABLED:
		phy_set_led(BOARD_LED_BLUE, false);
		phy_set_led(BOARD_LED_GREEN, true);
		break;

	case LED_STACKCREATED:
		phy_set_led(BOARD_LED_GREEN, true);
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_INIRQ:
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_SIGNAL:
		phy_set_led(BOARD_LED_GREEN, true);
		break;

	case LED_ASSERTION:
		phy_set_led(BOARD_LED_RED, true);
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_PANIC:
		phy_set_led(BOARD_LED_RED, true);
		break;

	case LED_IDLE : /* IDLE */
		phy_set_led(BOARD_LED_RED, true);
		break;
	}
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
	if (!nuttx_owns_leds) {
		return;
	}

	switch (led) {
	default:
		break;

	case LED_SIGNAL:
		phy_set_led(BOARD_LED_GREEN, false);
		break;

	case LED_INIRQ:
		phy_set_led(BOARD_LED_BLUE, false);
		break;

	case LED_ASSERTION:
		phy_set_led(BOARD_LED_RED, false);
		phy_set_led(BOARD_LED_BLUE, false);
		break;

	case LED_PANIC:
		phy_set_led(BOARD_LED_RED, false);
		break;

	case LED_IDLE : /* IDLE */
		phy_set_led(BOARD_LED_RED, false);
		break;
	}
}

#endif /* CONFIG_ARCH_LEDS */

/****************************************************************************
 * boards/arm/stm32h7/stm32h753xi-eval2/src/stm32_autoleds.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32h753xi-eval2.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x) (sizeof((x)) / sizeof((x)[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Indexed by BOARD_LED_<color> */

static const uint32_t g_ledmap[BOARD_NLEDS] =
{
  GPIO_LED_GREEN,
  GPIO_LED_RED,
};

static bool g_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void phy_set_led(int led, bool state)
{
  /* Active High */

  stm32_gpiowrite(g_ledmap[led], state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  int i;

  /* Configure the LD1 GPIO for output. Initial state is OFF */

  for (i = 0; i < ARRAYSIZE(g_ledmap); i++)
    {
      stm32_configgpio(g_ledmap[i]);
    }
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
    case LED_STARTED:
    case LED_HEAPALLOCATE:
    case LED_IRQSENABLED:
      break;

    case LED_STACKCREATED:
      phy_set_led(BOARD_LED_RED, true);
      g_initialized = true;
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
      break;

    case LED_ASSERTION:
      phy_set_led(BOARD_LED_RED, true);
      phy_set_led(BOARD_LED_GREEN, true);
      break;

    case LED_PANIC:
      phy_set_led(BOARD_LED_RED, true);
      break;

    case LED_IDLE:
      phy_set_led(BOARD_LED_GREEN, true);
      break;

    default:
      break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
    case LED_SIGNAL:
    case LED_INIRQ:
      break;

    case LED_ASSERTION:
      phy_set_led(BOARD_LED_RED, false);
      phy_set_led(BOARD_LED_GREEN, false);
      break;

    case LED_PANIC:
      phy_set_led(BOARD_LED_RED, false);
      break;

    case LED_IDLE:
      phy_set_led(BOARD_LED_GREEN, false);
    break;

    default:
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */

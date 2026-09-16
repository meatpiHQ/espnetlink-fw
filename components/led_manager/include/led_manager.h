/*
 * This file is part of the MeatPi components project.
 *
 * Copyright (C) 2022-2026 MeatPi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file led_manager.h
 * @brief WiCAN RGB status LED owner (service component).
 *
 * Owns the AW2023 I2C LED controller (shared bus via i2c_bus) and — more
 * importantly — the ARBITRATION over it: the LED is one shared resource
 * many components want ("I'm idle", "OTA in progress", "error"), so raw
 * color calls would fight. Instead, clients set an INDICATION at a fixed
 * priority; the highest occupied priority owns the LED. Clearing a
 * priority falls back to the next one down; the IDLE indication (from
 * settings, default solid blue) is always occupied, so the LED never goes
 * undefined.
 *
 * Example (the OTA case this was designed for): idle = solid blue at
 * PRIO_IDLE; ota_manager's session (via main's glue) sets fast-blinking
 * red at PRIO_CRITICAL — every lower-priority set during the update is
 * stored but not shown; when the session ends and CRITICAL clears, the
 * stored state below becomes visible again.
 *
 * Thread safety: all calls serialize on an internal mutex (the multi-
 * register AW2023 updates must be atomic; the i2c_master driver only locks
 * single transactions).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Fixed priority ladder, lowest first. Keep it small and semantic —
 *  a new use case picks the level that matches its urgency. */
typedef enum
{
    LED_MANAGER_PRIO_IDLE = 0,  /* settings-defined default (always set)  */
    LED_MANAGER_PRIO_STATUS,    /* normal activity (connected, traffic)   */
    LED_MANAGER_PRIO_ALERT,     /* needs attention (degraded, error)      */
    LED_MANAGER_PRIO_CRITICAL,  /* do-not-power-off (OTA flash write)     */
    LED_MANAGER_PRIO_COUNT
} led_manager_prio_t;

typedef enum
{
    LED_MANAGER_OFF = 0,
    LED_MANAGER_SOLID,
    LED_MANAGER_BLINK_SLOW,     /* ~0.5 s on / 0.5 s off (hardware timed) */
    LED_MANAGER_BLINK_FAST,     /* ~130 ms on / off                       */
} led_manager_mode_t;

typedef struct
{
    led_manager_mode_t mode;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_manager_state_t;

/** Register settings ("led_manager") + log descriptors. No bus traffic. */
esp_err_t led_manager_init(void);

/** Bring up the AW2023 and show the idle indication. Refuses
 *  ESP_ERR_INVALID_STATE when unconfigured (§4.3 step 5). */
esp_err_t led_manager_start(void);

/** LED off, arbitration table kept. */
esp_err_t led_manager_stop(void);

/** Set/replace the indication at @p prio (it becomes visible if no higher
 *  priority is occupied). */
esp_err_t led_manager_set(led_manager_prio_t prio,
                          const led_manager_state_t *state);

/** Release @p prio; the next lower occupied indication shows. Clearing
 *  IDLE restores the settings-defined idle (it can't be vacated). */
esp_err_t led_manager_clear(led_manager_prio_t prio);

/** Pre-settings direct color (2026-07-19): brings the chip up if needed
 *  and shows a solid color — the SAFE-MODE / boot-button-feedback path,
 *  usable before (or without) the settings pass and start(). Normal
 *  indications go through the arbiter (led_manager_set), never this. */
esp_err_t led_manager_boot_color(uint8_t r, uint8_t g, uint8_t b);

/** What the LED is showing right now (for status/tests). */
esp_err_t led_manager_active(led_manager_prio_t *prio,
                             led_manager_state_t *state);

/** Register the `led` CLI command with cmdline_manager. Called
 *  INTERNALLY on the settings boot apply when the `cli` setting is true
 *  (default) — main no longer wires it. */
esp_err_t led_manager_register_cli(void);

/** Register the /api/led routes (GET active indication; PUT/DELETE drive
 *  the ALERT priority — the user slot). Main calls this only in HTTP
 *  compositions (the *_register_http pattern). */
esp_err_t led_manager_register_http(void);

#ifdef __cplusplus
}
#endif

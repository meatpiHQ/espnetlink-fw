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
 * @file led_manager_private.h
 * @brief Internal contracts: the PURE arbitration policy (host-testable,
 *        no I2C/RTOS) and the AW2023 chip layer.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "led_manager.h"

/* ---- pure arbitration policy (led_manager_policy.c) ---------------------- */

typedef struct
{
    bool                occupied[LED_MANAGER_PRIO_COUNT];
    led_manager_state_t state[LED_MANAGER_PRIO_COUNT];
} lm_arbiter_t;

void lm_arbiter_reset(lm_arbiter_t *a);
esp_err_t lm_arbiter_set(lm_arbiter_t *a, int prio,
                         const led_manager_state_t *state);
esp_err_t lm_arbiter_clear(lm_arbiter_t *a, int prio);

/** Highest occupied indication; returns its priority, or -1 when none
 *  (out gets LED_MANAGER_OFF). */
int lm_arbiter_active(const lm_arbiter_t *a, led_manager_state_t *out);

/* ---- AW2023 chip layer (led_manager_aw2023.c) ------------------------------ */

esp_err_t lm_aw2023_init(void);                        /* attach + chip setup */
esp_err_t lm_aw2023_apply(const led_manager_state_t *s);
esp_err_t lm_aw2023_device_id(uint8_t *id);

/** Raw register read for diagnostics (`led -d`). Note ISR (0x02) is
 *  clear-on-read by hardware. */
esp_err_t lm_aw2023_read_reg(uint8_t reg, uint8_t *val);

/* event_manager glue (led_manager_events.c) — ESPNetLink does not
 * compose an event_manager: the file is excluded from the build and the
 * call site in led_manager.c is a no-op. WiCAN declaration, for
 * reference:
 *   void lm_events_register(void);
 */

/* ---- settings (led_manager_settings.c) -------------------------------------- */

/** Register the "led_manager" descriptor with settings_manager. */
esp_err_t lm_settings_register(void);

/** The settings-defined idle indication (boot-applied). */
const led_manager_state_t *lm_settings_idle(void);

bool lm_settings_enabled(void);       /* the LED is driven at all           */
bool lm_settings_is_configured(void); /* boot apply ran (standard §4.3)     */

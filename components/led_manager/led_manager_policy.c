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
 * @file led_manager_policy.c
 * @brief PURE indication arbitration — no I2C, no RTOS, host-testable.
 */
#include <string.h>

#include "led_manager_private.h"

void lm_arbiter_reset(lm_arbiter_t *a)
{
    memset(a, 0, sizeof(*a));
}

esp_err_t lm_arbiter_set(lm_arbiter_t *a, int prio,
                         const led_manager_state_t *state)
{
    if (prio < 0 || prio >= LED_MANAGER_PRIO_COUNT || state == NULL ||
        (unsigned)state->mode > LED_MANAGER_BLINK_FAST)
    {
        return ESP_ERR_INVALID_ARG;
    }

    a->occupied[prio] = true;
    a->state[prio] = *state;
    return ESP_OK;
}

esp_err_t lm_arbiter_clear(lm_arbiter_t *a, int prio)
{
    if (prio < 0 || prio >= LED_MANAGER_PRIO_COUNT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    a->occupied[prio] = false;
    return ESP_OK;
}

int lm_arbiter_active(const lm_arbiter_t *a, led_manager_state_t *out)
{
    for (int p = LED_MANAGER_PRIO_COUNT - 1; p >= 0; p--)
    {
        if (a->occupied[p])
        {
            *out = a->state[p];
            return p;
        }
    }

    *out = (led_manager_state_t){ .mode = LED_MANAGER_OFF };
    return -1;
}

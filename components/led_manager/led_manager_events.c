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
 * @file led_manager_events.c
 * @brief event_manager glue: `led.indicate {r,g,b,mode}` + `led.clear`
 *        — the ALERT arbitration slot, same as the REST/CLI surfaces
 *        (rules never touch STATUS/CRITICAL; the ladder stays honest).
 */
#include <string.h>

#include "event_manager.h"

#include "led_manager.h"

static esp_err_t act_indicate(const cJSON *with, const em_event_t *ev)
{
    (void)ev;

    const cJSON *r = cJSON_GetObjectItemCaseSensitive(with, "r");
    const cJSON *g = cJSON_GetObjectItemCaseSensitive(with, "g");
    const cJSON *b = cJSON_GetObjectItemCaseSensitive(with, "b");
    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(with, "mode");

    if (!cJSON_IsNumber(r) || !cJSON_IsNumber(g) || !cJSON_IsNumber(b))
    {
        return ESP_ERR_INVALID_ARG;
    }

    led_manager_state_t st =
    {
        .mode = LED_MANAGER_SOLID,
        .r = (uint8_t)r->valueint,
        .g = (uint8_t)g->valueint,
        .b = (uint8_t)b->valueint,
    };

    if (cJSON_IsString(mode))
    {
        if (strcmp(mode->valuestring, "off") == 0)
        {
            st.mode = LED_MANAGER_OFF;
        }
        else if (strcmp(mode->valuestring, "blink_slow") == 0)
        {
            st.mode = LED_MANAGER_BLINK_SLOW;
        }
        else if (strcmp(mode->valuestring, "blink_fast") == 0)
        {
            st.mode = LED_MANAGER_BLINK_FAST;
        }
    }

    return led_manager_set(LED_MANAGER_PRIO_ALERT, &st);
}

static esp_err_t act_clear(const cJSON *with, const em_event_t *ev)
{
    (void)with;
    (void)ev;
    return led_manager_clear(LED_MANAGER_PRIO_ALERT);
}

void lm_events_register(void)
{
    static const em_action_t INDICATE =
    {
        .name = "led.indicate",
        .params_schema =
            "{\"type\":\"object\",\"properties\":{"
            "\"r\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255},"
            "\"g\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255},"
            "\"b\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255},"
            "\"mode\":{\"type\":\"string\","
            "\"enum\":[\"solid\",\"off\",\"blink_slow\",\"blink_fast\"]}},"
            "\"required\":[\"r\",\"g\",\"b\"]}",
        .run = act_indicate,
    };
    static const em_action_t CLEAR =
    {
        .name = "led.clear",
        .params_schema = "{\"type\":\"object\",\"properties\":{}}",
        .run = act_clear,
    };

    (void)event_manager_register_action(&INDICATE);
    (void)event_manager_register_action(&CLEAR);
}

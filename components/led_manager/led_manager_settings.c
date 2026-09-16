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
 * @file led_manager_settings.c
 * @brief settings_manager descriptor for led_manager: field-table schema
 *        (source of truth for shape/ranges/defaults) and on_apply (parses
 *        into the boot-applied idle indication + enabled flag; registers
 *        the CLI, settings-gated per standard §6b).
 */
#include <string.h>

#include "settings_manager.h"

#include "led_manager.h" /* led_manager_register_cli (settings-gated) */
#include "led_manager_private.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("enabled", true),
    SETTINGS_STR_ENUM("idle_mode", "off,solid,blink_slow", "solid"),
    /* startup/idle color rgb(0,204,255) — brand cyan (meatpi 2026-07-19) */
    SETTINGS_INT("idle_r", 0, 255, 0),
    SETTINGS_INT("idle_g", 0, 255, 204),
    SETTINGS_INT("idle_b", 0, 255, 255),
    SETTINGS_BOOL("cli", true),
};

/* boot-applied settings */
static led_manager_state_t s_idle;
static bool s_enabled;
static bool s_configured;

const led_manager_state_t *lm_settings_idle(void)
{
    return &s_idle;
}

bool lm_settings_enabled(void)
{
    return s_enabled;
}

bool lm_settings_is_configured(void)
{
    return s_configured;
}

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(settings,
                                                         "enabled");

    s_enabled = cJSON_IsTrue(item);

    const char *mode = "solid";

    item = cJSON_GetObjectItemCaseSensitive(settings, "idle_mode");

    if (cJSON_IsString(item))
    {
        mode = item->valuestring;
    }

    s_idle.mode = (strcmp(mode, "off") == 0) ? LED_MANAGER_OFF :
                  (strcmp(mode, "blink_slow") == 0) ? LED_MANAGER_BLINK_SLOW :
                  LED_MANAGER_SOLID;

    item = cJSON_GetObjectItemCaseSensitive(settings, "idle_r");
    s_idle.r = (uint8_t)(cJSON_IsNumber(item) ? item->valueint : 0);
    item = cJSON_GetObjectItemCaseSensitive(settings, "idle_g");
    s_idle.g = (uint8_t)(cJSON_IsNumber(item) ? item->valueint : 204);
    item = cJSON_GetObjectItemCaseSensitive(settings, "idle_b");
    s_idle.b = (uint8_t)(cJSON_IsNumber(item) ? item->valueint : 255);

    /* CLI ownership: settings-gated self-registration (reboot-to-apply) */
    if (!cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(settings, "cli")))
    {
        static bool s_cli_registered;

        if (!s_cli_registered && led_manager_register_cli() == ESP_OK)
        {
            s_cli_registered = true;
        }
    }

    s_configured = true;
    return ESP_OK;
}

esp_err_t lm_settings_register(void)
{
    static const settings_descriptor_t DESC =
    {
        .name = "led_manager",
        .version = 1,
        .fields = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply = on_apply,
    };

    return settings_manager_register(&DESC);
}

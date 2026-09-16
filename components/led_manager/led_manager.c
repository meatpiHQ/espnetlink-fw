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
 * @file led_manager.c
 * @brief Lifecycle and the mutex-serialized public API around the pure
 *        arbiter + AW2023 layer. Settings live in led_manager_settings.c
 *        (standard §4.1).
 */
#include "led_manager.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "log_manager.h"

#include "led_manager_private.h"

static const char *TAG = "led_manager";

static lm_arbiter_t s_arb EXT_RAM_BSS_ATTR;
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf; /* internal: FreeRTOS object */
static bool s_started;

/* ---- lifecycle -------------------------------------------------------------- */

esp_err_t led_manager_init(void)
{
    static const log_descriptor_t LOG_DESC =
        { "led_manager", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC);

    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    lm_arbiter_reset(&s_arb);
    /* ESPNetLink does not compose an event_manager — the events glue
     * (led_manager_events.c) is excluded from the build. WiCAN calls
     * lm_events_register() here to declare led.indicate/led.clear. */
    return lm_settings_register();
}

/** Apply whatever the arbiter resolves to (caller holds s_lock). */
static esp_err_t apply_active(void)
{
    led_manager_state_t st;

    lm_arbiter_active(&s_arb, &st);
    return lm_aw2023_apply(&st);
}

esp_err_t led_manager_start(void)
{
    if (!lm_settings_is_configured())
    {
        ESP_LOGE(TAG, "unconfigured; not starting");
        return ESP_ERR_INVALID_STATE; /* standard §4.3 step 5 */
    }

    if (s_started)
    {
        return ESP_OK;
    }

    if (!lm_settings_enabled())
    {
        ESP_LOGI(TAG, "disabled in settings");
        return ESP_OK; /* started-but-dark: set/clear still book-keep */
    }

    esp_err_t err = lm_aw2023_init();

    if (err != ESP_OK)
    {
        return err;
    }

    const led_manager_state_t *idle = lm_settings_idle();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    lm_arbiter_set(&s_arb, LED_MANAGER_PRIO_IDLE, idle);
    err = apply_active();
    s_started = (err == ESP_OK);
    xSemaphoreGive(s_lock);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "started (idle %s %u,%u,%u)",
                 (idle->mode == LED_MANAGER_OFF) ? "off" :
                 (idle->mode == LED_MANAGER_BLINK_SLOW) ? "blink" : "solid",
                 idle->r, idle->g, idle->b);
    }

    return err;
}

esp_err_t led_manager_stop(void)
{
    if (!s_started)
    {
        return ESP_OK;
    }

    led_manager_state_t off = { .mode = LED_MANAGER_OFF };

    xSemaphoreTake(s_lock, portMAX_DELAY);
    lm_aw2023_apply(&off);
    s_started = false;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

/* ---- public API --------------------------------------------------------------- */

esp_err_t led_manager_set(led_manager_prio_t prio,
                          const led_manager_state_t *state)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    esp_err_t err = lm_arbiter_set(&s_arb, prio, state);

    if (err == ESP_OK && s_started)
    {
        err = apply_active();
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t led_manager_clear(led_manager_prio_t prio)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    esp_err_t err = lm_arbiter_clear(&s_arb, prio);

    if (err == ESP_OK && prio == LED_MANAGER_PRIO_IDLE &&
        lm_settings_is_configured())
    {
        /* IDLE can't be vacated: restore the settings-defined idle */
        lm_arbiter_set(&s_arb, LED_MANAGER_PRIO_IDLE, lm_settings_idle());
    }

    if (err == ESP_OK && s_started)
    {
        err = apply_active();
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t led_manager_active(led_manager_prio_t *prio,
                             led_manager_state_t *state)
{
    if (prio == NULL || state == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    int p = lm_arbiter_active(&s_arb, state);

    xSemaphoreGive(s_lock);
    *prio = (p < 0) ? LED_MANAGER_PRIO_IDLE : (led_manager_prio_t)p;
    return (p < 0) ? ESP_ERR_INVALID_STATE : ESP_OK;
}

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
 * @file led_manager_aw2023.c
 * @brief AW2023 3-channel LED controller layer (port of the field-proven
 *        legacy led.c onto the shared i2c_bus / i2c_master driver).
 *
 * Channel map (WiCAN Pro): LED0 = red, LED1 = green, LED2 = blue.
 * Blink uses the chip's hardware pattern engine (T1..T4 timers), so a
 * blinking indication costs zero CPU after the register writes.
 */
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "i2c_bus.h"

#include "led_manager.h"
#include "led_manager_private.h"

static const char *TAG = "led_manager";

#define AW2023_ADDR       0x45
#define AW2023_SPEED_HZ   400000
#define AW2023_TIMEOUT_MS 100

/* register map */
#define AW2023_RSTR  0x00
#define AW2023_GCR1  0x01
#define AW2023_LCTR  0x30
#define AW2023_LCFG0 0x31 /* +1 green, +2 blue                             */
#define AW2023_PWM0  0x34 /* +1 green, +2 blue                             */
#define AW2023_LED0T0 0x37 /* T1/T2; +1 = T3/T4; +2 = T0/repeat; +3/ch     */

#define AW2023_MD_BIT (1 << 4) /* LCFGx: pattern mode                      */

static i2c_master_dev_handle_t s_dev;

/* chip time codes: 0=0ms 1=130ms 2=260ms 3=380ms 4=510ms ...               */
#define T_NONE 0x0
#define T_130MS 0x1
#define T_510MS 0x4

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    return i2c_master_transmit(s_dev, buf, sizeof(buf),
                               AW2023_TIMEOUT_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1,
                                       AW2023_TIMEOUT_MS);
}

esp_err_t lm_aw2023_init(void)
{
    esp_err_t err = i2c_bus_add_device(AW2023_ADDR, AW2023_SPEED_HZ,
                                       &s_dev);

    if (err != ESP_OK)
    {
        return err;
    }

    /* legacy bring-up sequence (field-proven values) */
    reg_write(AW2023_RSTR, 0x55);          /* software reset               */
    vTaskDelay(pdMS_TO_TICKS(2));
    err = reg_write(AW2023_GCR1, 0x01);    /* enable chip                  */

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "AW2023 not responding: %s", esp_err_to_name(err));
        return err;
    }

    /* standby->active: the internal OSC needs a beat before the LED
       registers accept writes. The legacy 200 kHz legacy-i2c stack was
       slow enough by accident; this 400 kHz i2c_master path slammed
       LCTR immediately and the write was silently LOST (chip ACKs in
       standby but ignores) — LED dark forever (bench 2026-07-19,
       `led -d`: LCTR=0x00 with everything else applied). Delay, then
       verify the critical channel-enable write actually landed. */
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t lctr = 0;

    for (int attempt = 0; attempt < 3; attempt++)
    {
        reg_write(AW2023_LCTR, 0x07);      /* enable all channels          */

        if (reg_read(AW2023_LCTR, &lctr) == ESP_OK && (lctr & 0x07) == 0x07)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if ((lctr & 0x07) != 0x07)
    {
        ESP_LOGE(TAG, "AW2023 channel enable never stuck (LCTR=0x%02X)",
                 lctr);
    }

    reg_write(AW2023_LCFG0, 0x03);         /* per-channel current          */
    reg_write(AW2023_LCFG0 + 1, 0x03);
    reg_write(AW2023_LCFG0 + 2, 0x03);
    return ESP_OK;
}

esp_err_t lm_aw2023_device_id(uint8_t *id)
{
    if (s_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return reg_read(AW2023_RSTR, id);
}

esp_err_t lm_aw2023_read_reg(uint8_t reg, uint8_t *val)
{
    if (s_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return reg_read(reg, val);
}

esp_err_t led_manager_boot_color(uint8_t r, uint8_t g, uint8_t b)
{
    /* pre-settings path (safe mode / boot-hold feedback): bring the chip
       up if the normal lifecycle hasn't, then show a solid color. Safe
       mode never runs the settings pass, so this must not depend on it. */
    if (s_dev == NULL)
    {
        esp_err_t err = lm_aw2023_init();

        if (err != ESP_OK)
        {
            return err;
        }
    }

    led_manager_state_t s =
        { .mode = LED_MANAGER_SOLID, .r = r, .g = g, .b = b };

    return lm_aw2023_apply(&s);
}

/** One pattern timing per mode; applied identically to the channels that
 *  are lit so a mixed color blinks as one (T1=T3=0: hard on/off). */
static esp_err_t pattern_write(int ch, uint8_t hold, uint8_t off)
{
    uint8_t base = AW2023_LED0T0 + (uint8_t)(ch * 3);
    esp_err_t err = reg_write(base, (uint8_t)((T_NONE << 4) | hold));

    if (err == ESP_OK)
    {
        err = reg_write(base + 1, (uint8_t)((T_NONE << 4) | off));
    }

    if (err == ESP_OK)
    {
        err = reg_write(base + 2, 0x00); /* no delay, repeat forever       */
    }

    return err;
}

esp_err_t lm_aw2023_apply(const led_manager_state_t *s)
{
    if (s_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* self-heal the channel enables: LCTR can come up empty (init-race,
       see lm_aw2023_init) or be cleared by a chip-level event — without
       LE0..2 every PWM write below is invisible. Applies are rare, so
       one read is cheap; preserve FREQ/EXP. */
    uint8_t lctr = 0;

    if (reg_read(AW2023_LCTR, &lctr) == ESP_OK && (lctr & 0x07) != 0x07)
    {
        (void)reg_write(AW2023_LCTR, (uint8_t)(lctr | 0x07));
    }

    uint8_t level[3] = { s->r, s->g, s->b };
    bool blink = (s->mode == LED_MANAGER_BLINK_SLOW ||
                  s->mode == LED_MANAGER_BLINK_FAST);
    uint8_t t = (s->mode == LED_MANAGER_BLINK_FAST) ? T_130MS : T_510MS;
    esp_err_t err = ESP_OK;

    for (int ch = 0; ch < 3 && err == ESP_OK; ch++)
    {
        uint8_t lcfg;

        err = reg_read(AW2023_LCFG0 + (uint8_t)ch, &lcfg);

        if (err != ESP_OK)
        {
            break;
        }

        bool lit = (s->mode != LED_MANAGER_OFF) && (level[ch] > 0);

        if (blink && lit)
        {
            err = pattern_write(ch, t, t);

            if (err == ESP_OK)
            {
                err = reg_write(AW2023_LCFG0 + (uint8_t)ch,
                                lcfg | AW2023_MD_BIT);
            }
        }
        else
        {
            err = reg_write(AW2023_LCFG0 + (uint8_t)ch,
                            lcfg & (uint8_t)~AW2023_MD_BIT);
        }

        if (err == ESP_OK)
        {
            err = reg_write(AW2023_PWM0 + (uint8_t)ch,
                            (s->mode == LED_MANAGER_OFF) ? 0 : level[ch]);
        }
    }

    return err;
}

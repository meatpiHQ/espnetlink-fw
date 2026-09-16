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
 * @file led_manager_http.c
 * @brief The /api/led routes (§9.1: feature components register their own
 *        domain routes) — main calls led_manager_register_http() only in
 *        HTTP compositions, so the LED code carries no HTTP dependency.
 *
 * The REST client drives the ALERT priority — the same user-facing slot
 * the `led -c` CLI command uses. Firmware-internal indications (STATUS,
 * OTA's CRITICAL) are never writable from outside: the arbiter's ladder
 * stays honest.
 */
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "http_server_manager.h"

#include "led_manager.h"

static const char *TAG = "led_manager";

static const char *mode_name(led_manager_mode_t mode)
{
    switch (mode)
    {
        case LED_MANAGER_SOLID:      return "solid";
        case LED_MANAGER_BLINK_SLOW: return "blink_slow";
        case LED_MANAGER_BLINK_FAST: return "blink_fast";
        default:                     return "off";
    }
}

static const char *prio_name(led_manager_prio_t prio)
{
    switch (prio)
    {
        case LED_MANAGER_PRIO_STATUS:   return "status";
        case LED_MANAGER_PRIO_ALERT:    return "alert";
        case LED_MANAGER_PRIO_CRITICAL: return "critical";
        default:                        return "idle";
    }
}

/* GET /api/led — what the LED is showing right now (the arbiter's winner). */
static esp_err_t led_get_handler(httpd_req_t *req)
{
    led_manager_prio_t  prio;
    led_manager_state_t state;
    char body[128];

    if (led_manager_active(&prio, &state) == ESP_OK)
    {
        snprintf(body, sizeof(body),
                 "{\"priority\":\"%s\",\"mode\":\"%s\","
                 "\"r\":%u,\"g\":%u,\"b\":%u}",
                 prio_name(prio), mode_name(state.mode),
                 state.r, state.g, state.b);
    }
    else
    {
        snprintf(body, sizeof(body), "{\"priority\":null}");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_simple(httpd_req_t *req, const char *status,
                             const char *body)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

/* PUT /api/led {"r":0-255,"g":..,"b":..[,"mode":"solid|blink_slow|
   blink_fast|off"]} — set the ALERT indication (the user slot). */
static esp_err_t led_put_handler(httpd_req_t *req)
{
    char buf[160];
    int  len = httpd_req_recv(req, buf,
                              (req->content_len < sizeof(buf) - 1)
                                  ? req->content_len : sizeof(buf) - 1);

    if (len <= 0)
    {
        return send_simple(req, "400 Bad Request",
                           "{\"error\":\"missing body\"}");
    }

    buf[len] = '\0';

    cJSON *in = cJSON_Parse(buf);
    const cJSON *r = cJSON_GetObjectItemCaseSensitive(in, "r");
    const cJSON *g = cJSON_GetObjectItemCaseSensitive(in, "g");
    const cJSON *b = cJSON_GetObjectItemCaseSensitive(in, "b");
    const cJSON *m = cJSON_GetObjectItemCaseSensitive(in, "mode");

    if (!cJSON_IsNumber(r) || !cJSON_IsNumber(g) || !cJSON_IsNumber(b) ||
        r->valueint < 0 || r->valueint > 255 ||
        g->valueint < 0 || g->valueint > 255 ||
        b->valueint < 0 || b->valueint > 255)
    {
        cJSON_Delete(in);
        return send_simple(req, "400 Bad Request",
                           "{\"error\":\"need r,g,b 0-255\"}");
    }

    led_manager_state_t state =
    {
        .mode = LED_MANAGER_SOLID,
        .r    = (uint8_t)r->valueint,
        .g    = (uint8_t)g->valueint,
        .b    = (uint8_t)b->valueint,
    };

    if (cJSON_IsString(m))
    {
        if (strcmp(m->valuestring, "off") == 0)
        {
            state.mode = LED_MANAGER_OFF;
        }
        else if (strcmp(m->valuestring, "blink_slow") == 0)
        {
            state.mode = LED_MANAGER_BLINK_SLOW;
        }
        else if (strcmp(m->valuestring, "blink_fast") == 0)
        {
            state.mode = LED_MANAGER_BLINK_FAST;
        }
        else if (strcmp(m->valuestring, "solid") != 0)
        {
            cJSON_Delete(in);
            return send_simple(req, "400 Bad Request",
                               "{\"error\":\"mode: off|solid|blink_slow|"
                               "blink_fast\"}");
        }
    }

    cJSON_Delete(in);

    if (led_manager_set(LED_MANAGER_PRIO_ALERT, &state) != ESP_OK)
    {
        return send_simple(req, "500 Internal Server Error",
                           "{\"error\":\"led unavailable\"}");
    }

    return send_simple(req, "200 OK", "{\"ok\":true}");
}

/* DELETE /api/led — release the ALERT indication (arbiter falls back). */
static esp_err_t led_delete_handler(httpd_req_t *req)
{
    if (led_manager_clear(LED_MANAGER_PRIO_ALERT) != ESP_OK)
    {
        return send_simple(req, "500 Internal Server Error",
                           "{\"error\":\"led unavailable\"}");
    }

    return send_simple(req, "200 OK", "{\"ok\":true}");
}

esp_err_t led_manager_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/led", .method = HTTP_GET,
          .handler = led_get_handler },
        { .uri = "/api/led", .method = HTTP_PUT,
          .handler = led_put_handler },
        { .uri = "/api/led", .method = HTTP_DELETE,
          .handler = led_delete_handler },
    };

    esp_err_t err = http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "/api/led registered");
    }

    return err;
}

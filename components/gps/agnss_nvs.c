/*
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/**
 * @file agnss_nvs.c
 * @brief NVS-backed position cache for AGNSS assistance.
 *
 * Stores last known GPS position (lat, lon, alt) in NVS namespace "gnss"
 * for injection into the LC76G on next boot.
 */

#include "agnss_internal.h"

#include <math.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "agnss_nvs";

#define NVS_NAMESPACE "gnss"
#define NVS_KEY_LAT   "lat"
#define NVS_KEY_LON   "lon"
#define NVS_KEY_ALT   "alt"

/* NVS doesn't support double natively; store as int64 with fixed-point scaling.
 * 1e7 gives ~0.01 m resolution for lat/lon; altitude stored as cm. */
#define COORD_SCALE   10000000LL
#define ALT_SCALE     100LL

esp_err_t agnss_nvs_read_position(double *lat, double *lon, double *alt)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK)
    {
        ESP_LOGD(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    int64_t i_lat, i_lon, i_alt;
    bool ok = true;

    if (nvs_get_i64(h, NVS_KEY_LAT, &i_lat) != ESP_OK)
    {
        ok = false;
    }
    if (nvs_get_i64(h, NVS_KEY_LON, &i_lon) != ESP_OK)
    {
        ok = false;
    }
    if (nvs_get_i64(h, NVS_KEY_ALT, &i_alt) != ESP_OK)
    {
        ok = false;
    }

    nvs_close(h);

    if (!ok)
    {
        return ESP_ERR_NOT_FOUND;
    }

    *lat = (double)i_lat / (double)COORD_SCALE;
    *lon = (double)i_lon / (double)COORD_SCALE;
    *alt = (double)i_alt / (double)ALT_SCALE;

    ESP_LOGD(TAG, "Read position: lat=%.6f lon=%.6f alt=%.1f", *lat, *lon, *alt);
    return ESP_OK;
}

esp_err_t agnss_nvs_write_position(double lat, double lon, double alt)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    int64_t i_lat = (int64_t)llround(lat * (double)COORD_SCALE);
    int64_t i_lon = (int64_t)llround(lon * (double)COORD_SCALE);
    int64_t i_alt = (int64_t)llround(alt * (double)ALT_SCALE);

    bool ok = true;
    if (nvs_set_i64(h, NVS_KEY_LAT, i_lat) != ESP_OK)
    {
        ok = false;
    }
    if (nvs_set_i64(h, NVS_KEY_LON, i_lon) != ESP_OK)
    {
        ok = false;
    }
    if (nvs_set_i64(h, NVS_KEY_ALT, i_alt) != ESP_OK)
    {
        ok = false;
    }

    if (ok)
    {
        err = nvs_commit(h);
    }
    else
    {
        err = ESP_FAIL;
    }

    nvs_close(h);

    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "Wrote position: lat=%.6f lon=%.6f alt=%.1f", lat, lon, alt);
    }
    else
    {
        ESP_LOGE(TAG, "NVS write failed: %s", esp_err_to_name(err));
    }

    return err;
}

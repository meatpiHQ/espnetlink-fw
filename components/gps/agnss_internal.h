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
 * @file agnss_internal.h
 * @brief Internal shared declarations for the agnss component.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compute standard NMEA XOR checksum over a sentence (between '$' and '*').
 */
uint8_t agnss_nmea_checksum(const char *sentence);

/**
 * Build a complete PAIR command string with checksum and CRLF.
 *
 * @param buf      Output buffer.
 * @param buf_len  Size of output buffer.
 * @param body     Command body WITHOUT leading '$' or trailing '*XX\r\n'.
 *                 Example: "PAIR590,2025,6,14,12,34,56"
 * @return Number of bytes written (excluding NUL), or -1 on error.
 */
int agnss_build_pair_cmd(char *buf, size_t buf_len, const char *body);

/* ---- Injection status (implemented in agnss.c) ---- */

void agnss_get_status(bool *time_injected, bool *position_injected);

/* ---- NVS position cache (implemented in agnss_nvs.c) ---- */

esp_err_t agnss_nvs_read_position(double *lat, double *lon, double *alt);
esp_err_t agnss_nvs_write_position(double lat, double lon, double alt);

#ifdef __cplusplus
}
#endif

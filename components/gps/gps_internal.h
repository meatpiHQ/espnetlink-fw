/**
 * @file gps_internal.h
 * @brief Component-internal API between the gps core (gps.c) and its CLI
 *        (gps_cli.c). Not part of the public header.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GPS module power rail (GPS_PWR_EN_PIN). */
esp_err_t gps_power_pin_set(bool enabled);
bool gps_power_pin_is_enabled(void);

/* NMEA copy on UART0 (GPS_UART0_OUT routing). */
esp_err_t gps_uart0_prepare(void);
void gps_set_uart0_output(bool enabled);
bool gps_is_uart0_output_enabled(void);

#ifdef __cplusplus
}
#endif

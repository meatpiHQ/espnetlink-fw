#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Snapshot of the last parsed GPS fix (thread-safe copy). */
typedef struct
{
    bool     has_fix;
    bool     valid;
    double   lat;
    double   lon;
    int      satellites;       /* tracked (from GGA) */
    int      sats_in_view;     /* visible (from GSV) */
    int      fix_quality;      /* 0=none, 1=GPS, 2=DGPS (from GGA) */
    int      fix_type;         /* 1=none, 2=2D, 3=3D (from GSA)    */
    double   altitude_m;
    double   hdop;
    double   pdop;
    double   vdop;
    double   speed_knots;
    double   speed_kmph;
    double   course_deg;
    uint32_t last_update_ms;
} gps_fix_snapshot_t;

// Initialize GPS UART + parser task (idempotent).
// If enable_agnss is true, AGNSS assistance (EASY, time/position injection)
// will be started in the background after the GPS module boots.
esp_err_t gps_init(bool enable_agnss);

/**
 * Copy the latest GPS fix into *out.  Thread-safe.
 * Returns ESP_OK if fix data was copied, ESP_ERR_INVALID_STATE if no fix yet.
 */
esp_err_t gps_get_fix(gps_fix_snapshot_t *out);

// Enable/disable console streaming mode (raw NMEA sentences).
void gps_set_streaming(bool enabled);
bool gps_is_streaming(void);

/**
 * Check if the GPS UART is receiving data.
 * Clears the RX event flag, waits up to timeout_ms for new data.
 * @return true if data was received within the timeout.
 */
bool gps_is_functional(uint32_t timeout_ms);

/**
 * Wait for a PAIR response line from the LC76G.
 *
 * The GPS parser task intercepts lines starting with "$PAIR" and routes
 * them to an internal queue.  This function blocks until a matching line
 * arrives or the timeout expires.
 *
 * @param out       Buffer to receive the NUL-terminated line (incl. CRLF stripped).
 * @param out_len   Size of out buffer.
 * @param timeout_ms Maximum wait in milliseconds.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout.
 */
esp_err_t gps_pair_response_wait(char *out, size_t out_len, uint32_t timeout_ms);

/**
 * Write raw bytes to the GPS UART.
 * Use this instead of calling uart_write_bytes(GPS_UART_PORT, …) directly
 * so writes are serialised with the parser task.
 */
esp_err_t gps_uart_write(const void *data, size_t len);

// Register the `gps` console command (idempotent).
// This is called by usb_cli_console so new components can add commands.
void gps_console_register(void);

/** Register `GET /api/gps` (web UI status page; §9.1 feature route).
 *  Called by main in HTTP-bearing compositions. */
esp_err_t gps_register_http(void);

/* -----------------------------------------------------------------------
 * AGNSS assistance (EASY prediction + time/position injection)
 * ----------------------------------------------------------------------- */

/**
 * Save the LC76G EASY navigation data to flash.
 * Call before powering down the system.
 */
esp_err_t agnss_save_easy_data(void);

/**
 * Attempt to inject reference time into the LC76G now.
 * Succeeds only if the system clock is marked as synced.
 * Safe to call even if time was already injected (no-op in that case).
 */
esp_err_t agnss_try_inject_time(void);

/**
 * Register the `agnss` CLI command.
 * Called by usb_cli_console during console initialisation.
 */
void agnss_console_register(void);

/* -----------------------------------------------------------------------
 * Settings (standard §4.1: descriptor "gps" in gps_settings.c)
 * ----------------------------------------------------------------------- */

/** Boot-applied configuration (reboot-to-apply; §4.2). */
typedef struct
{
    bool enabled;       /* legacy GPS_ENABLED (reserved; not actuated yet) */
    bool passthru;      /* legacy GPS_PASSTHRU  — NMEA passthrough console */
    bool uart0_out;     /* legacy GPS_UART0_OUT — NMEA copy on UART0       */
    bool agnss_enabled; /* legacy AGNSS_ENABLED                            */
    int  agnss_cache_s; /* legacy AGNSS_CACHE_S — position cache interval  */
    bool cli;           /* §6b gate for this component's console commands  */
    int  epoc_constellation; /* PAIR498 mask from "epoc_constellation":
                                1=GPS, 3=GPS+Galileo, 5=GPS+BDS (the only
                                configs the LC76G EPOC supports)            */
} gps_settings_t;

/** Register the "gps" settings descriptor. Called by main between
 *  settings_manager_init() and settings_manager_start(). */
esp_err_t gps_register_settings(void);

/** Boot-applied settings (valid after the settings apply pass). */
const gps_settings_t *gps_settings(void);
bool gps_settings_is_configured(void);

/** Persist one field (validate + store; applies NEXT boot — §4.2).
 *  Used by the gps/agnss CLI commands. */
esp_err_t gps_settings_persist_bool(const char *field, bool value);
esp_err_t gps_settings_persist_int(const char *field, int value);

/** Read one field's PENDING (persisted) value — what the next boot will
 *  apply. Query paths use these so a set-then-get sequence behaves like
 *  the legacy in-memory config did. */
esp_err_t gps_settings_pending_bool(const char *field, bool *out);
esp_err_t gps_settings_pending_int(const char *field, int *out);

#ifdef __cplusplus
}
#endif

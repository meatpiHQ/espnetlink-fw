/**
 * @file mdns_manager_private.h
 * @brief Internal contract: the PURE string builders (host-testable).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/** 6 MAC bytes -> "AA:BB:CC:DD:EE:FF" (the legacy TXT `mac` format —
 *  colon-separated UPPERCASE; HA uses it as the stable unique ID).
 *  False when @p out can't hold 18 bytes. */
bool mm_format_mac(const uint8_t mac[6], char *out, size_t cap);

/** "espnetlink_<id>" / "espnetlink_<id>.local" (ESPNetLink port of the
 *  WiCAN "wican_<id>" scheme). False on overflow. */
bool mm_build_hostname(const char *id, char *out, size_t cap);
bool mm_build_hostname_local(const char *id, char *out, size_t cap);

/* ---- settings (mdns_manager_settings.c) -------------------------------------- */

/** Register the "mdns_manager" descriptor with settings_manager. */
esp_err_t mdns_manager_settings_register(void);

bool mdns_manager_settings_enabled(void);       /* the advertisement runs at all      */
bool mdns_manager_settings_is_configured(void); /* boot apply ran (standard §4.3)     */

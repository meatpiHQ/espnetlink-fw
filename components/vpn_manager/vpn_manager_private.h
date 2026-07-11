/**
 * @file vpn_manager_private.h
 * @brief Internals shared between the state machine, the esp_wireguard
 *        glue, the pure config checks, and the HTTP/CLI/events glue.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "vpn_manager.h"

/* boot-applied configuration (strings sized to the settings schema) */
typedef struct
{
    bool enabled;
    bool tailscale;            /* type == "tailscale"                */
    char private_key[64];
    char peer_public_key[64];
    char preshared_key[64];
    char address[32];          /* local tunnel IP, optional /cidr    */
    char allowed_ip[32];
    char allowed_ip_mask[32];
    char endpoint[64];
    int  port;
    int  keepalive_s;
    bool default_route;
    char dns[16];              /* optional override, "" = off        */
    /* tailscale (microlink) */
    char ts_auth_key[96];      /* SECRET (auth_key redaction suffix) */
    char ts_device_name[48];   /* "" = wican_<device_id>             */
    char ts_control_url[64];   /* "" = tailscale.com; host only      */
} vpn_config_t;

/* ---- vpn_manager_check.c — PURE (host-tested) ---------------------------- */

/* a WireGuard key is 32 bytes base64: exactly 44 chars, valid alphabet,
 * '=' terminated */
bool vpn_check_wg_key(const char *key_b64);
/* endpoint hostname/IP sanity (len 1..63, [A-Za-z0-9.-:_]) */
bool vpn_check_endpoint(const char *host);
/* full enabled-config validation; returns NULL when OK, else a static
 * human-readable reason (the settings on_validate message) */
const char *vpn_check_config(const vpn_config_t *cfg);

/* ---- vpn_manager_wg.c — esp_wireguard glue (state task ONLY) ------------- */

esp_err_t vpn_wg_up(const vpn_config_t *cfg);   /* init + connect      */
void      vpn_wg_down(void);                    /* disconnect + reset  */
bool      vpn_wg_peer_up(void);
esp_err_t vpn_wg_set_default_route(void);

/* ---- vpn_manager_ts.c — tailscale/microlink glue (state task ONLY) ------- */

esp_err_t vpn_ts_up(const vpn_config_t *cfg);   /* init + start        */
void      vpn_ts_down(void);                    /* stop + destroy      */
bool      vpn_ts_connected(void);
bool      vpn_ts_failed(void);
void      vpn_ts_status(char *ip, size_t ip_len, int *peers);

/* ---- settings (vpn_manager_settings.c) ------------------------------------ */

/** Register the "vpn_manager" descriptor with settings_manager. */
esp_err_t vpn_settings_register(void);

/** Boot-applied config; valid once vpn_settings_is_configured(). */
const vpn_config_t *vpn_settings_config(void);
bool vpn_settings_is_configured(void); /* boot apply ran (standard §4.3) */

/** Display endpoint ("host:port" or the control server), derived on
 *  the settings apply for status/log strings. */
const char *vpn_settings_endpoint(void);

/* ---- glue ----------------------------------------------------------------- */

/* ESPNetLink does not compose an event_manager: the events glue
 * (vpn_manager_events.c) is excluded from the build and its call sites
 * in vpn_manager.c are no-ops. WiCAN declarations, for reference:
 *   void vpn_events_register(void);
 *   void vpn_events_state(bool connected);
 */

/**
 * @file wifi_manager_private.h
 * @brief Internals shared between wifi_manager.c, wifi_manager_settings.c and
 *        the pure candidate-selection module (wifi_manager_select.c — keep
 *        that one free of IDF deps so the host tests compile it directly).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WM_SSID_LEN       33 /* 32 chars + NUL */
#define WM_PASS_LEN       65 /* 64 chars + NUL */
#define WM_HOSTNAME_LEN   33
#define WM_MAX_FALLBACKS  5
#define WM_MAX_CANDIDATES (1 + WM_MAX_FALLBACKS)

/* Ban list: SSIDs that repeatedly fail auth get skipped for a while.
 * A ban means "prefer anything else", NOT "stop trying": when a banned
 * network is the only candidate available it is still retried once per
 * WM_BANNED_RETRY_MS (meatpi 2026-07-08 — the same SSID name can carry
 * a different password at another location; the device must reconnect
 * promptly once back in range of the right one). */
#define WM_AUTH_FAIL_THRESHOLD 3
#define WM_BAN_DURATION_MS     600000u /* 10 minutes */
#define WM_BANNED_RETRY_MS     60000u  /* banned-only trickle cadence */

typedef enum
{
    WM_MODE_OFF = 0,
    WM_MODE_STA,
    WM_MODE_AP,
    WM_MODE_APSTA,
} wm_mode_t;

typedef enum
{
    WM_PS_NONE = 0,
    WM_PS_MIN,
    WM_PS_MAX,
} wm_ps_t;

/* WiFi memory profile (v5): trades internal RAM for WiFi throughput by
 * sizing the driver buffers at esp_wifi_init. */
typedef enum
{
    WM_RAM_FULL = 0, /* IDF defaults — max throughput          */
    WM_RAM_LEAN,     /* smaller buffers, ~15 KB internal freed */
    WM_RAM_CUSTOM,   /* the wifi_static_rx/tx + cache_tx knobs */
} wm_ram_profile_t;

/* AP security selection (v4). AUTO keeps the historic behaviour: WPA2-PSK
 * when a password is set, OPEN when it is blank. */
typedef enum
{
    WM_AP_AUTH_AUTO = 0,
    WM_AP_AUTH_OPEN,
    WM_AP_AUTH_WPA2,
    WM_AP_AUTH_WPA2WPA3,
    WM_AP_AUTH_WPA3,
} wm_ap_auth_t;

typedef struct
{
    char ssid[WM_SSID_LEN];
    char password[WM_PASS_LEN];
} wm_network_t;

/* v6: one network's STA addressing (host-order; is_static=false = DHCP) */
typedef struct
{
    bool     is_static;
    uint32_t ip;      /* valid when is_static             */
    uint32_t netmask;
    uint32_t gw;      /* 0 = none (isolated LAN is legal) */
} wm_addr_t;

/** Boot-applied configuration (from the settings descriptor). */
typedef struct
{
    wm_mode_t    mode;
    wm_network_t sta[WM_MAX_CANDIDATES]; /* [0] primary, then fallbacks     */
    size_t       sta_count;              /* 0 => no STA network configured  */
    char         hostname[WM_HOSTNAME_LEN];
    bool         sta_auto_reconnect;
    int32_t      sta_max_retry;          /* -1 => infinite                  */
    wm_network_t ap;
    uint8_t      ap_channel;
    uint8_t      ap_max_connections;
    bool         ap_auto_disable;        /* drop AP once STA has an IP      */
    uint32_t     ap_ip;                  /* v4: AP gateway IP (host order);
                                            /24 assumed. 0 => legacy default
                                            192.168.80.1                    */
    bool         ap_hidden;              /* v4: don't beacon the SSID       */
    bool         ap_ht40;                /* v4: 40 MHz bandwidth (else 20)  */
    wm_ap_auth_t ap_auth;                /* v4: security mode (see enum)    */
    wm_ps_t      power_save;
    bool         trusted[WM_MAX_CANDIDATES]; /* v3: untrusted nets
                                            refuse inbound admin
                                            arriving via STA        */
    uint32_t     sta_roam_interval_s;    /* while connected to a FALLBACK,
                                            re-check for a higher-priority
                                            network this often; 0 = never
                                            (v2, TASK: home > car hotspot) */
    /* v5: WiFi memory profile — buffer counts fed to esp_wifi_init.
       0 in a count = "use the profile/IDF default". */
    wm_ram_profile_t ram_profile;
    uint16_t     wifi_static_rx;         /* custom: static RX buffers  */
    uint16_t     wifi_static_tx;         /* custom: static TX buffers  */
    uint16_t     wifi_cache_tx;          /* custom: cache TX buffers   */
    /* v6: PER-NETWORK STA addressing — static (user-fixed) or DHCP
       (default). Each candidate lives on its own LAN, so its addressing
       travels with it through the empty-slot collapse (like trusted[]).
       Applied per CONNECT ATTEMPT (apply_sta_network). Host-order.
       sta_dns = GLOBAL custom DNS override for BOTH modes (0 =
       automatic: DHCP-provided, or that network's gateway when static). */
    wm_addr_t    sta_addr[WM_MAX_CANDIDATES];
    uint32_t     sta_dns;
} wm_config_t;

/** Parse a dotted-quad IPv4 into host-order uint32 (any address). */
bool wm_parse_ipv4(const char *s, uint32_t *out);

/** wm_parse_ipv4 + reject a host octet of 0/255 (network/broadcast) —
 *  for the device's own / gateway addresses. Shared with the settings
 *  validator (wifi_manager_settings.c) and the host tests. */
bool wm_parse_ap_ipv4(const char *s, uint32_t *out);

/** True when @p mask (host order) is a contiguous netmask (/1../31). */
bool wm_netmask_valid(uint32_t mask);

/* ---- pure candidate selection + ban list (wifi_manager_select.c) --------- */

typedef struct
{
    char     ssid[WM_SSID_LEN];
    uint8_t  fail_count;
    uint32_t banned_until_ms; /* 0 => not banned */
} wm_ban_entry_t;

typedef struct
{
    wm_ban_entry_t bans[WM_MAX_CANDIDATES * 2];
    int            seq_cursor;         /* rotation state, no-scan path  */
    uint32_t       last_banned_try_ms; /* banned-only retry throttle;
                                          0 = none yet                  */
} wm_select_state_t;

/** Reset selection state; makes the first sequential pick the primary. */
void wm_select_init(wm_select_state_t *st);

/**
 * Pick a candidate given scan results: the primary if present and not
 * banned, else the first present un-banned fallback (priority order).
 * When everything visible is banned, a banned candidate is returned at
 * most once per WM_BANNED_RETRY_MS (else -1) — banned means "prefer
 * anything else", never "stop trying". @p present is an array of SSIDs
 * seen in the scan. Returns candidate index or -1 (nothing eligible
 * this cycle).
 */
int wm_select_from_scan(wm_select_state_t *st, const wm_network_t *cand,
                        size_t cand_count,
                        const char (*present)[WM_SSID_LEN],
                        size_t present_count, uint32_t now_ms);

/**
 * No-scan/blind path (single network, scan failed, or nothing visible
 * matched — hidden SSIDs and scan-truncation land here): rotate through
 * the candidates, skipping banned ones; all-banned falls back to the
 * same WM_BANNED_RETRY_MS throttle. Returns index or -1.
 */
int wm_select_sequential(wm_select_state_t *st, const wm_network_t *cand,
                         size_t cand_count, uint32_t now_ms);

/**
 * Roam-to-preferred check (call while CONNECTED to candidate
 * @p current_idx > 0): returns the highest-priority candidate index
 * < current_idx that is visible in @p present and not banned, or -1
 * (stay). The "home > car hotspot" migration decision.
 */
int wm_select_better(wm_select_state_t *st, const wm_network_t *cand,
                     size_t cand_count, int current_idx,
                     const char (*present)[WM_SSID_LEN],
                     size_t present_count, uint32_t now_ms);

/** Record an auth-related failure; bans after WM_AUTH_FAIL_THRESHOLD. */
void wm_select_on_auth_fail(wm_select_state_t *st, const char *ssid,
                            uint32_t now_ms);

/** Successful connect: clear failure count / ban for @p ssid. */
void wm_select_on_success(wm_select_state_t *st, const char *ssid);

/** True while @p ssid is inside its ban window. */
bool wm_select_is_banned(const wm_select_state_t *st, const char *ssid,
                         uint32_t now_ms);

/* Escalating reconnect backoff (2026-07-10): how many 5 s reconnect
 * loops to SKIP after real attempt number @p retry_count failed.
 * 0 for the first two attempts (fast candidate walk / short outages),
 * then 1/3/5 (10/20/30 s cadence — cap per meatpi) so a parked device
 * doesn't scan every 5 s forever. Resets with sta_retry_count on
 * got-ip. */
#define WM_BACKOFF_MAX_SKIP_LOOPS 5 /* x 5 s = 30 s cadence cap */
int wm_backoff_skip_loops(int32_t retry_count);

/* ---- runtime interface suspension (wifi_manager_suspend.c) -----------------
 * EPHEMERAL per-interface suspension driven by interface_manager's policy
 * (BLE/WiFi arbitration). Settings untouched; a reboot resets. */

/** True while the policy holds STA / AP suspended (gates the reconnect
 *  task and the AP-restore paths in wifi_manager.c). */
bool wm_suspend_sta_active(void);
bool wm_suspend_ap_active(void);

/** wifi_manager.c internals the suspend layer needs. */
bool wm_core_started(void);

/* ---- settings glue (wifi_manager_settings.c) ------------------------------ */

/** Register the "wifi_manager" descriptor with settings_manager. */
esp_err_t wm_settings_register(void);

/** Boot-applied config; valid once wm_settings_is_configured(). */
const wm_config_t *wm_settings_config(void);
bool wm_settings_is_configured(void);

#ifdef __cplusplus
}
#endif

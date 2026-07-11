/**
 * @file wifi_manager_settings.c
 * @brief settings_manager descriptor for wifi_manager: JSON schema (source of
 *        truth for shape/ranges/defaults), on_apply (stores config — runs at
 *        boot before wifi_manager_start(), touches no hardware), on_validate.
 *
 * Fallback networks are flat keys (fallback1_ssid/.._password ... 5) because
 * the settings schema subset validates flat properties (no arrays).
 */
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"

#include "dev_status_manager.h"
#include "settings_manager.h"
#include "wifi_manager.h" /* wifi_manager_register_cli (settings-gated) */

#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

/* Field table -> the manager generates the JSON Schema at registration
   (settings_manager.h). One row per field, one source of truth (§5). */
/* clang-format off */
static const settings_field_t WM_FIELDS[] =
{
    /* shipping default = AP (meatpi 2026-07-05): a fresh device is an
       access point for onboarding; STA comes from user configuration */
    SETTINGS_STR_ENUM("mode",             "off,sta,ap,apsta", "ap"),
    SETTINGS_STR ("sta_ssid",           32, ""),
    SETTINGS_STR ("sta_password",       64, ""),
    /* v3: per-network trust — untrusted networks refuse ALL inbound
       admin surfaces (API/UI/WS) that arrive VIA the STA address;
       outbound clients (autopid HTTP posts, MQTT) are unaffected and
       the device's own AP + USB stay fully usable (meatpi 2026-07-08:
       shared/office networks must not expose configuration) */
    SETTINGS_BOOL("sta_trusted",        true),
    SETTINGS_STR ("hostname",           32, ""),
    SETTINGS_BOOL("sta_auto_reconnect", true),
    SETTINGS_INT ("sta_max_retry",      -1, 1000, -1),
    /* v6: PER-NETWORK STA addressing — DHCP (default) or static; each
       network (primary + every fallback) carries its own choice since
       they live on different LANs. sta_dns = GLOBAL DNS override for
       BOTH modes (empty = automatic: DHCP-provided, or that network's
       gateway when static). */
    SETTINGS_STR_ENUM("sta_ip_mode",    "dhcp,static", "dhcp"),
    SETTINGS_STR ("sta_static_ip",      15, ""),
    SETTINGS_STR ("sta_static_netmask", 15, "255.255.255.0"),
    SETTINGS_STR ("sta_static_gw",      15, ""),
    SETTINGS_STR ("sta_dns",            15, ""),
    SETTINGS_STR_ENUM("fallback1_ip_mode", "dhcp,static", "dhcp"),
    SETTINGS_STR ("fallback1_static_ip",      15, ""),
    SETTINGS_STR ("fallback1_static_netmask", 15, "255.255.255.0"),
    SETTINGS_STR ("fallback1_static_gw",      15, ""),
    SETTINGS_STR_ENUM("fallback2_ip_mode", "dhcp,static", "dhcp"),
    SETTINGS_STR ("fallback2_static_ip",      15, ""),
    SETTINGS_STR ("fallback2_static_netmask", 15, "255.255.255.0"),
    SETTINGS_STR ("fallback2_static_gw",      15, ""),
    SETTINGS_STR_ENUM("fallback3_ip_mode", "dhcp,static", "dhcp"),
    SETTINGS_STR ("fallback3_static_ip",      15, ""),
    SETTINGS_STR ("fallback3_static_netmask", 15, "255.255.255.0"),
    SETTINGS_STR ("fallback3_static_gw",      15, ""),
    SETTINGS_STR_ENUM("fallback4_ip_mode", "dhcp,static", "dhcp"),
    SETTINGS_STR ("fallback4_static_ip",      15, ""),
    SETTINGS_STR ("fallback4_static_netmask", 15, "255.255.255.0"),
    SETTINGS_STR ("fallback4_static_gw",      15, ""),
    SETTINGS_STR_ENUM("fallback5_ip_mode", "dhcp,static", "dhcp"),
    SETTINGS_STR ("fallback5_static_ip",      15, ""),
    SETTINGS_STR ("fallback5_static_netmask", 15, "255.255.255.0"),
    SETTINGS_STR ("fallback5_static_gw",      15, ""),
    SETTINGS_STR ("fallback1_ssid",     32, ""),
    SETTINGS_STR ("fallback1_password", 64, ""),
    SETTINGS_BOOL("fallback1_trusted",  true),
    SETTINGS_STR ("fallback2_ssid",     32, ""),
    SETTINGS_STR ("fallback2_password", 64, ""),
    SETTINGS_BOOL("fallback2_trusted",  true),
    SETTINGS_STR ("fallback3_ssid",     32, ""),
    SETTINGS_STR ("fallback3_password", 64, ""),
    SETTINGS_BOOL("fallback3_trusted",  true),
    SETTINGS_STR ("fallback4_ssid",     32, ""),
    SETTINGS_STR ("fallback4_password", 64, ""),
    SETTINGS_BOOL("fallback4_trusted",  true),
    SETTINGS_STR ("fallback5_ssid",     32, ""),
    SETTINGS_STR ("fallback5_password", 64, ""),
    SETTINGS_BOOL("fallback5_trusted",  true),
    SETTINGS_STR ("ap_ssid",            32, ""),
    SETTINGS_STR_LEN("ap_password",      8, 64, "12345678"),
    SETTINGS_INT ("ap_channel",          1, 13, 6),
    SETTINGS_INT ("ap_max_connections",  1, 10, 4),
    SETTINGS_BOOL("ap_auto_disable",    false),
    /* v4: AP LAN knobs. ap_ip is the device/gateway address on its own
       network (a /24 is assumed; the DHCP pool follows it). Default keeps
       the legacy 192.168.80.1 that existing apps/docs hardcode and that
       dodges the common 192.168.0/1.x home LANs. ap_auth "auto" = the
       historic rule (WPA2 with a password, OPEN without). */
    SETTINGS_STR ("ap_ip",              15, "192.168.80.1"),
    SETTINGS_BOOL("ap_hidden",          false),
    SETTINGS_STR_ENUM("ap_bandwidth",   "ht20,ht40", "ht20"),
    SETTINGS_STR_ENUM("ap_auth",        "auto,open,wpa2,wpa2wpa3,wpa3",
                      "auto"),
    SETTINGS_STR_ENUM("power_save",     "none,min,max", "none"),
    /* v2: while connected to a FALLBACK network, re-scan this often and
       migrate when a higher-priority one (e.g. home) is visible; 0=off */
    SETTINGS_INT ("sta_roam_interval_s", 0, 86400, 300),
    /* v5: WiFi memory profile (runtime — the buffer counts are
       wifi_init_config_t fields, applied at esp_wifi_init; reboot to
       take effect). `full` = IDF defaults (throughput); `lean` frees
       ~15 KB internal at a throughput cost (the RAM-cliff Option B, e.g.
       to run WiFi + BLE together); `custom` uses the three numeric knobs
       below. The knobs only apply when profile = custom; their ranges
       match the IDF Kconfig bounds so no combination is invalid. */
    SETTINGS_STR_ENUM("wifi_ram_profile", "full,lean,custom", "full"),
    SETTINGS_INT ("wifi_static_rx",      2, 25, 10),
    SETTINGS_INT ("wifi_static_tx",      1, 64, 8),
    SETTINGS_INT ("wifi_cache_tx",       0, 128, 32),
    SETTINGS_BOOL("cli",                true),
};
/* clang-format on */

static wm_config_t s_config EXT_RAM_BSS_ATTR; /* boot-applied; PSRAM (§2) */
static bool        s_configured;

static void copy_str(char *dst, size_t dst_len, const cJSON *obj,
                     const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);

    dst[0] = '\0';

    if (cJSON_IsString(item) && item->valuestring != NULL)
    {
        strncpy(dst, item->valuestring, dst_len - 1);
        dst[dst_len - 1] = '\0';
    }
}

static int get_int(const cJSON *obj, const char *key, int fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);

    return cJSON_IsNumber(item) ? item->valueint : fallback;
}

static bool get_bool(const cJSON *obj, const char *key, bool fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);

    return cJSON_IsBool(item) ? cJSON_IsTrue(item) : fallback;
}

static wm_mode_t parse_mode(const cJSON *obj)
{
    char mode[8];

    copy_str(mode, sizeof(mode), obj, "mode");

    if (strcmp(mode, "off") == 0)
    {
        return WM_MODE_OFF;
    }

    if (strcmp(mode, "sta") == 0)
    {
        return WM_MODE_STA;
    }

    if (strcmp(mode, "ap") == 0)
    {
        return WM_MODE_AP;
    }

    return WM_MODE_APSTA;
}

static wm_ps_t parse_power_save(const cJSON *obj)
{
    char ps[8];

    copy_str(ps, sizeof(ps), obj, "power_save");

    if (strcmp(ps, "min") == 0)
    {
        return WM_PS_MIN;
    }

    if (strcmp(ps, "max") == 0)
    {
        return WM_PS_MAX;
    }

    return WM_PS_NONE;
}

static bool parse_ap_bandwidth(const cJSON *obj)
{
    char bw[8];

    copy_str(bw, sizeof(bw), obj, "ap_bandwidth");
    return strcmp(bw, "ht40") == 0;
}

static wm_ram_profile_t parse_ram_profile(const cJSON *obj)
{
    char p[8];

    copy_str(p, sizeof(p), obj, "wifi_ram_profile");

    if (strcmp(p, "lean") == 0)
    {
        return WM_RAM_LEAN;
    }

    if (strcmp(p, "custom") == 0)
    {
        return WM_RAM_CUSTOM;
    }

    return WM_RAM_FULL;
}

static wm_ap_auth_t parse_ap_auth(const cJSON *obj)
{
    char a[12];

    copy_str(a, sizeof(a), obj, "ap_auth");

    if (strcmp(a, "open") == 0)
    {
        return WM_AP_AUTH_OPEN;
    }

    if (strcmp(a, "wpa2") == 0)
    {
        return WM_AP_AUTH_WPA2;
    }

    if (strcmp(a, "wpa2wpa3") == 0)
    {
        return WM_AP_AUTH_WPA2WPA3;
    }

    if (strcmp(a, "wpa3") == 0)
    {
        return WM_AP_AUTH_WPA3;
    }

    return WM_AP_AUTH_AUTO;
}

/** Default AP SSID: unique per device, derived from THE device id
 *  (dev_status_manager, 12 hex chars): product prefix + last 3 MAC bytes,
 *  uppercase (espnetlink port of the WiCAN_ scheme). */
static void derive_ap_ssid(char *dst, size_t dst_len)
{
    const char *id = dev_status_manager_device_id(); /* "aabbccddeeff" */
    size_t len = strlen(id);
    char tail[7] = "";

    for (size_t i = 0; len >= 6 && i < 6; i++)
    {
        char c = id[len - 6 + i];

        tail[i] = (char)((c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c);
    }

    snprintf(dst, dst_len, "ESPNetLink_%s", tail);
}

/** v6: parse one network's addressing keys ("<base>_ip_mode",
 *  "<base>_static_ip/_netmask/_gw"). Parse failures fall back to DHCP
 *  rather than a half-configured static netif (on_validate is strict). */
static void parse_sta_addr(const cJSON *settings, const char *base,
                           wm_addr_t *a)
{
    char key[32], buf[16];

    memset(a, 0, sizeof(*a));
    snprintf(key, sizeof(key), "%s_ip_mode", base);
    copy_str(buf, sizeof(buf), settings, key);
    a->is_static = (strcmp(buf, "static") == 0);
    snprintf(key, sizeof(key), "%s_static_ip", base);
    copy_str(buf, sizeof(buf), settings, key);

    if (!wm_parse_ap_ipv4(buf, &a->ip))
    {
        a->ip = 0;
        a->is_static = false;
    }

    snprintf(key, sizeof(key), "%s_static_netmask", base);
    copy_str(buf, sizeof(buf), settings, key);

    if (!wm_parse_ipv4(buf, &a->netmask) || !wm_netmask_valid(a->netmask))
    {
        a->netmask = 0xFFFFFF00u; /* /24 */
    }

    snprintf(key, sizeof(key), "%s_static_gw", base);
    copy_str(buf, sizeof(buf), settings, key);

    if (!wm_parse_ap_ipv4(buf, &a->gw))
    {
        a->gw = 0;
    }
}

static esp_err_t wm_on_apply(const cJSON *settings)
{
    wm_config_t *c = &s_config;

    memset(c, 0, sizeof(*c));

    c->mode = parse_mode(settings);

    /* STA candidate list: primary first, then non-empty fallbacks in order */
    char fb_key[24];

    copy_str(c->sta[0].ssid, WM_SSID_LEN, settings, "sta_ssid");
    copy_str(c->sta[0].password, WM_PASS_LEN, settings, "sta_password");
    c->trusted[0] = get_bool(settings, "sta_trusted", true);
    parse_sta_addr(settings, "sta", &c->sta_addr[0]);
    c->sta_count = (c->sta[0].ssid[0] != '\0') ? 1 : 0;

    for (int i = 1; i <= WM_MAX_FALLBACKS; i++)
    {
        wm_network_t net;

        snprintf(fb_key, sizeof(fb_key), "fallback%d_ssid", i);
        copy_str(net.ssid, WM_SSID_LEN, settings, fb_key);
        snprintf(fb_key, sizeof(fb_key), "fallback%d_password", i);
        copy_str(net.password, WM_PASS_LEN, settings, fb_key);

        if (net.ssid[0] != '\0' && c->sta_count < WM_MAX_CANDIDATES)
        {
            /* the trust flag + addressing travel with their network
               through the empty-slot collapse */
            snprintf(fb_key, sizeof(fb_key), "fallback%d_trusted", i);
            c->trusted[c->sta_count] = get_bool(settings, fb_key, true);
            snprintf(fb_key, sizeof(fb_key), "fallback%d", i);
            parse_sta_addr(settings, fb_key, &c->sta_addr[c->sta_count]);
            c->sta[c->sta_count++] = net;
        }
    }

    copy_str(c->hostname, WM_HOSTNAME_LEN, settings, "hostname");
    c->sta_auto_reconnect = get_bool(settings, "sta_auto_reconnect", true);
    c->sta_max_retry = get_int(settings, "sta_max_retry", -1);

    /* v6: global DNS override (validated in on_validate) */
    char ipbuf[16];

    copy_str(ipbuf, sizeof(ipbuf), settings, "sta_dns");

    if (!wm_parse_ipv4(ipbuf, &c->sta_dns))
    {
        c->sta_dns = 0;
    }

    copy_str(c->ap.ssid, WM_SSID_LEN, settings, "ap_ssid");

    if (c->ap.ssid[0] == '\0')
    {
        derive_ap_ssid(c->ap.ssid, WM_SSID_LEN);
    }

    copy_str(c->ap.password, WM_PASS_LEN, settings, "ap_password");
    c->ap_channel = (uint8_t)get_int(settings, "ap_channel", 6);
    c->ap_max_connections = (uint8_t)get_int(settings, "ap_max_connections", 4);
    c->ap_auto_disable = get_bool(settings, "ap_auto_disable", false);

    char ap_ip[16] = "";

    copy_str(ap_ip, sizeof(ap_ip), settings, "ap_ip");
    c->ap_ip = 0; /* apply falls back to the legacy default on 0 */
    (void)wm_parse_ap_ipv4(ap_ip, &c->ap_ip); /* validated in on_validate */
    c->ap_hidden = get_bool(settings, "ap_hidden", false);
    c->ap_ht40 = parse_ap_bandwidth(settings);
    c->ap_auth = parse_ap_auth(settings);
    c->power_save = parse_power_save(settings);
    c->sta_roam_interval_s =
        (uint32_t)get_int(settings, "sta_roam_interval_s", 300);

    c->ram_profile = parse_ram_profile(settings);
    c->wifi_static_rx = (uint16_t)get_int(settings, "wifi_static_rx", 10);
    c->wifi_static_tx = (uint16_t)get_int(settings, "wifi_static_tx", 8);
    c->wifi_cache_tx = (uint16_t)get_int(settings, "wifi_cache_tx", 32);

    s_configured = true;

    /* CLI ownership: the component registers its own commands, gated by
       its `cli` setting (reboot-to-apply) — main no longer wires this */
    if (get_bool(settings, "cli", true))
    {
        static bool s_cli_registered;

        if (!s_cli_registered && wifi_manager_register_cli() == ESP_OK)
        {
            s_cli_registered = true;
        }
    }

    /* §10: don't log secrets — SSIDs only. */
    ESP_LOGI(TAG, "config applied: mode=%d sta_networks=%u ap_ssid=%s",
             (int)c->mode, (unsigned)c->sta_count, c->ap.ssid);

    return ESP_OK;
}

static esp_err_t wm_on_validate(const cJSON *settings, char *err,
                                size_t err_len)
{
    /* STA modes with a password but no SSID are a config mistake the schema
     * can't express (cross-field). */
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(settings, "sta_ssid");
    const cJSON *pass = cJSON_GetObjectItemCaseSensitive(settings,
                                                         "sta_password");

    bool ssid_empty = !cJSON_IsString(ssid) || ssid->valuestring == NULL ||
                      ssid->valuestring[0] == '\0';
    bool pass_set = cJSON_IsString(pass) && pass->valuestring != NULL &&
                    pass->valuestring[0] != '\0';

    if (ssid_empty && pass_set)
    {
        snprintf(err, err_len, "sta_password set but sta_ssid is empty");
        return ESP_ERR_INVALID_ARG;
    }

    /* AP IP must be a usable /24 gateway address (v4) */
    const cJSON *ap_ip = cJSON_GetObjectItemCaseSensitive(settings, "ap_ip");

    if (cJSON_IsString(ap_ip) && ap_ip->valuestring != NULL &&
        ap_ip->valuestring[0] != '\0')
    {
        uint32_t parsed;

        if (!wm_parse_ap_ipv4(ap_ip->valuestring, &parsed))
        {
            snprintf(err, err_len,
                     "ap_ip '%s' is not a valid gateway address "
                     "(dotted IPv4, host octet 1..254)",
                     ap_ip->valuestring);
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* v6: static addressing needs a coherent set — per NETWORK (the
       primary + every fallback carries its own choice) */
    static const char *const BASES[] =
    {
        "sta", "fallback1", "fallback2", "fallback3", "fallback4",
        "fallback5",
    };

    for (size_t b = 0; b < sizeof(BASES) / sizeof(BASES[0]); b++)
    {
        char key[32];
        uint32_t parsed;

        snprintf(key, sizeof(key), "%s_ip_mode", BASES[b]);

        const cJSON *im = cJSON_GetObjectItemCaseSensitive(settings, key);

        if (!cJSON_IsString(im) || im->valuestring == NULL ||
            strcmp(im->valuestring, "static") != 0)
        {
            continue;
        }

        snprintf(key, sizeof(key), "%s_static_ip", BASES[b]);

        const cJSON *v = cJSON_GetObjectItemCaseSensitive(settings, key);

        if (!cJSON_IsString(v) || v->valuestring == NULL ||
            !wm_parse_ap_ipv4(v->valuestring, &parsed))
        {
            snprintf(err, err_len,
                     "%s_ip_mode 'static' needs a valid %s_static_ip "
                     "(dotted IPv4, host octet 1..254)",
                     BASES[b], BASES[b]);
            return ESP_ERR_INVALID_ARG;
        }

        snprintf(key, sizeof(key), "%s_static_netmask", BASES[b]);
        v = cJSON_GetObjectItemCaseSensitive(settings, key);

        if (cJSON_IsString(v) && v->valuestring != NULL &&
            v->valuestring[0] != '\0' &&
            (!wm_parse_ipv4(v->valuestring, &parsed) ||
             !wm_netmask_valid(parsed)))
        {
            snprintf(err, err_len,
                     "%s_static_netmask '%s' is not a valid netmask",
                     BASES[b], v->valuestring);
            return ESP_ERR_INVALID_ARG;
        }

        snprintf(key, sizeof(key), "%s_static_gw", BASES[b]);
        v = cJSON_GetObjectItemCaseSensitive(settings, key);

        if (cJSON_IsString(v) && v->valuestring != NULL &&
            v->valuestring[0] != '\0' &&
            !wm_parse_ap_ipv4(v->valuestring, &parsed))
        {
            snprintf(err, err_len,
                     "%s_static_gw '%s' is not a valid gateway address",
                     BASES[b], v->valuestring);
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* v6: the DNS override applies in BOTH ip modes — validate whenever set */
    {
        uint32_t parsed;
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(settings,
                                                          "sta_dns");

        if (cJSON_IsString(v) && v->valuestring != NULL &&
            v->valuestring[0] != '\0' &&
            !wm_parse_ipv4(v->valuestring, &parsed))
        {
            snprintf(err, err_len,
                     "sta_dns '%s' is not a valid IPv4 address",
                     v->valuestring);
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* an encrypted AP mode needs a password (WPA*-PSK); the schema can't
       express this cross-field rule */
    const cJSON *auth = cJSON_GetObjectItemCaseSensitive(settings, "ap_auth");
    const cJSON *ap_pass = cJSON_GetObjectItemCaseSensitive(settings,
                                                            "ap_password");
    bool ap_pass_set = cJSON_IsString(ap_pass) &&
                       ap_pass->valuestring != NULL &&
                       ap_pass->valuestring[0] != '\0';

    if (cJSON_IsString(auth) && auth->valuestring != NULL &&
        strcmp(auth->valuestring, "open") != 0 &&
        strcmp(auth->valuestring, "auto") != 0 && !ap_pass_set)
    {
        snprintf(err, err_len,
                 "ap_auth '%s' needs an ap_password", auth->valuestring);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t wm_on_migrate(uint32_t from_version, cJSON *settings)
{
    (void)from_version; /* v1->v2 +sta_roam_interval_s, v2->v3 +trusted
                           flags, v3->v4 +ap_ip/hidden/bandwidth/auth,
                           v4->v5 +wifi_ram_profile, v5->v6 +STA static
                           addressing; fill-missing defaults cover all */
    (void)settings;
    return ESP_OK;
}

esp_err_t wm_settings_register(void)
{
    static const settings_descriptor_t desc =
    {
        .name          = "wifi_manager",
        .version       = 6, /* v6: +STA static addressing; v5:
                               +wifi_ram_profile; v4: +AP LAN knobs;
                               v3: +trusted flags; v2: +roam */
        .fields        = WM_FIELDS,
        .field_count   = sizeof(WM_FIELDS) / sizeof(WM_FIELDS[0]),
        .defaults_json = NULL,
        .on_apply      = wm_on_apply,
        .on_validate   = wm_on_validate,
        .on_migrate    = wm_on_migrate,
    };

    return settings_manager_register(&desc);
}

const wm_config_t *wm_settings_config(void)
{
    return &s_config;
}

bool wm_settings_is_configured(void)
{
    return s_configured;
}

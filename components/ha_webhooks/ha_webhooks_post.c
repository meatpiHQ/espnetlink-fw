/**
 * @file ha_webhooks_post.c
 * @brief The poster task (PSRAM stack): builds {schema, status, gps}
 *        (device-contract v2, ESPNetLink sections) and POSTs it to the
 *        HA webhook URL(s) with failover. status.device_id/fw_version/
 *        hw_version are guaranteed in EVERY push; HTTP 403 from HA
 *        (identity rejection) pauses posting after HW_REJECT_LIMIT
 *        consecutive cycles until the next registration.
 *
 * ESPNetLink deviation from the WiCAN original: the autopid_data/config
 * sections are replaced by a `gps` section (this product pushes
 * status + gps, ADDING_A_DEVICE.md A3), and `status` carries the LTE
 * link fields instead of battery/VPN/BLE.
 *
 * PSRAM stack is safe because every payload source is RAM:
 *   - gps           -> gps_get_fix()                (RAM snapshot)
 *   - status        -> dev_status/lte getters       (RAM)
 * and the outbound HTTP goes through http_client_manager (its own task).
 * Stats updates are cache-only (no flash) — §2 corollary.
 */
#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "miniz.h" /* the ROM deflate compressor (tdefl_* are ROM symbols) */

#include "cJSON.h"

#include "dev_status_manager.h"
#include "gps.h"
#include "http_client_manager.h"
#include "lte_upstream_pppos.h"
#include "mdns_manager.h"
#include "sdkconfig.h"
#include "vpn_manager.h"

#include "ha_webhooks_private.h"

static const char *TAG = "ha_webhooks";

/* 8192 words = 32 KB (PSRAM, cheap): 24 KB overflowed once gzip landed —
 * the ROM tdefl deflate keeps its Huffman/blocking locals on the CALLER's
 * stack (several KB under an already-deep poster frame). Bench-verify
 * stack_hw via /api/status/tasks after any change here. */
#define HW_STACK_WORDS 8192
#define HW_HTTP_TIMEOUT_MS 6000

/* device-contract v2: the webhook payload shape version (bump only on
 * breaking changes; HA treats a missing key as 1) */
#define HW_PAYLOAD_SCHEMA 1

/* consecutive HTTP-403 cycles (HA identity rejection) before pushes
 * pause — the contract's "stop; needs user attention" device action.
 * Re-registration (POST /api/webhook) or reboot resumes. */
#define HW_REJECT_LIMIT 3

static TaskHandle_t     s_task;
static StaticTask_t     s_tcb;
static StackType_t      s_stack[HW_STACK_WORDS] EXT_RAM_BSS_ATTR;
static volatile bool    s_run;

/* previous snapshot for "changed" diffing (PSRAM string). gps is never
 * diffed: a partial {latitude-only} diff would corrupt the HA tracker. */
static char *s_prev_status EXT_RAM_BSS_ATTR;
static volatile bool s_resync; /* next build = full snapshot */

/* consecutive identity-rejected (403) cycles; >= HW_REJECT_LIMIT pauses
   the poster until the next registration (resync) or reboot */
static volatile uint32_t s_reject_streak;

void hw_poster_resync(void)
{
    s_resync = true;
    s_reject_streak = 0; /* a (re-)registration ends the rejected pause */
}

static void drop_prev_caches(void)
{
    free(s_prev_status);
    s_prev_status = NULL;
}

/* ---- status section (mirror of /api/status + device_id) ------------------- */

static cJSON *build_status(void)
{
    cJSON *s = cJSON_CreateObject();

    if (s == NULL)
    {
        return NULL;
    }

    /* device_id: the HA integration binds/validates on this (coordinator
       _validate_device_identity) — REQUIRED */
    cJSON_AddStringToObject(s, "device_id",
                            dev_status_manager_device_id());

    /* identity for the device registry, device-type inference and the
       update entity (contract v2 guarantees these in every push — the
       build_payload overlay keeps them alive through diff mode) */
    cJSON_AddStringToObject(s, "fw_version",
                            dev_status_manager_app_version());
    cJSON_AddStringToObject(s, "hw_version", CONFIG_WICAN_HW_VERSION);
    cJSON_AddStringToObject(s, "device_type", CONFIG_WICAN_DEVICE_TYPE);

    const char *host = mdns_manager_hostname();

    if (host != NULL && host[0] != '\0')
    {
        char url[48];

        snprintf(url, sizeof(url), "http://%s", host);
        cJSON_AddStringToObject(s, "mdns", url); /* HA configuration_url */
    }

    cJSON *bits = cJSON_AddObjectToObject(s, "bits");

    for (int i = 0; i < 24; i++)
    {
        EventBits_t bit = (EventBits_t)1u << i;
        const char *name = dev_status_manager_bit_name(bit);

        if (strcmp(name, "unknown") != 0)
        {
            cJSON_AddBoolToObject(bits, name,
                                  dev_status_manager_is_set(bit));
        }
    }

    cJSON_AddBoolToObject(s, "network_connected",
                          dev_status_manager_any_set(
                              DEV_STATUS_NETWORK_CONNECTED_MASK));

    char uptime[HW_TS_LEN] = "";

    dev_status_manager_format_uptime(uptime, sizeof(uptime));
    cJSON_AddStringToObject(s, "uptime", uptime);
    cJSON_AddStringToObject(s, "version",
                            dev_status_manager_app_version());

    float temp_c = 0;

    if (dev_status_manager_temperature(&temp_c) == ESP_OK)
    {
        cJSON_AddNumberToObject(s, "temp_c",
                                (double)((int)(temp_c * 10)) / 10.0);
    }

    /* the integration's fixed sensor entities (attributes.py) */
    bool sta = dev_status_manager_is_set(DEV_STATUS_BIT_STA_ENABLED);
    bool ap = dev_status_manager_is_set(DEV_STATUS_BIT_AP_ENABLED);

    cJSON_AddStringToObject(s, "wifi_mode",
                            (sta && ap) ? "AP+Station"
                            : sta       ? "Station"
                            : ap        ? "AP"
                                        : "Off");

    /* ESPNetLink product fields: the LTE uplink state (scalar values per
       contract rule 4 — these become sensors) */
    lte_status_t lte;

    if (lte_upstream_pppos_get_status(&lte) == ESP_OK && lte.valid)
    {
        cJSON_AddBoolToObject(s, "lte_connected", lte.ppp_connected);
        cJSON_AddBoolToObject(s, "lte_attached", lte.attached);

        if (lte.rssi != 99) /* 99 = unknown per 3GPP */
        {
            cJSON_AddNumberToObject(s, "lte_rssi_dbm", lte.rssi_dbm);
        }

        if (lte.operator_name[0] != '\0')
        {
            cJSON_AddStringToObject(s, "lte_operator", lte.operator_name);
        }

        if (lte.network_type[0] != '\0')
        {
            cJSON_AddStringToObject(s, "lte_network_type",
                                    lte.network_type);
        }
    }

    /* vpn_ip is HA's away-from-home control endpoint (it registers the
       tunnel address as the backup for /api/webhook + buttons) */
    vpn_manager_status_t vs;

    if (vpn_manager_status(&vs) == ESP_OK)
    {
        static const char *VPN_STATES[] =
        { "disabled", "waiting", "connecting", "connected" };

        cJSON_AddStringToObject(s, "vpn_status", VPN_STATES[vs.state]);

        char tip[32];

        if (vpn_manager_tunnel_ip(tip, sizeof(tip)) == ESP_OK)
        {
            cJSON_AddStringToObject(s, "vpn_ip", tip);
        }
    }

    return s;
}

/* ---- gps section (contract A3: ESPNetLink pushes status + gps) ------------- */

static cJSON *build_gps(void)
{
    gps_fix_snapshot_t fix;

    if (gps_get_fix(&fix) != ESP_OK || !fix.valid || !fix.has_fix)
    {
        return NULL; /* no fix -> omit the section (HA keeps last position) */
    }

    cJSON *g = cJSON_CreateObject();

    if (g == NULL)
    {
        return NULL;
    }

    cJSON_AddNumberToObject(g, "latitude", fix.lat);
    cJSON_AddNumberToObject(g, "longitude", fix.lon);

    /* contract: accuracy is an int in meters — approximated from HDOP
       (HDOP x ~5 m nominal GPS point error) */
    if (fix.hdop > 0)
    {
        int acc = (int)(fix.hdop * 5.0 + 0.5);

        cJSON_AddNumberToObject(g, "accuracy", acc < 1 ? 1 : acc);
    }

    cJSON_AddNumberToObject(g, "altitude", fix.altitude_m);
    cJSON_AddNumberToObject(g, "speed",
                            fix.speed_kmph / 3.6); /* contract: m/s */
    cJSON_AddNumberToObject(g, "heading", fix.course_deg);
    return g;
}

/* ---- diff: keys in curr that are new or changed vs prev ------------------- */

static cJSON *diff_object(const cJSON *curr, const char *prev_json)
{
    cJSON *out = cJSON_CreateObject();

    if (out == NULL)
    {
        return NULL;
    }

    cJSON *prev = prev_json != NULL ? cJSON_Parse(prev_json) : NULL;
    const cJSON *it = NULL;

    cJSON_ArrayForEach(it, curr)
    {
        const char *key = it->string;

        if (key == NULL)
        {
            continue;
        }

        bool changed = true;
        const cJSON *p = prev != NULL
                             ? cJSON_GetObjectItemCaseSensitive(prev, key)
                             : NULL;

        if (p != NULL)
        {
            changed = !cJSON_Compare(it, p, false);
        }

        if (changed)
        {
            cJSON_AddItemToObject(out, key, cJSON_Duplicate(it, true));
        }
    }

    if (prev != NULL)
    {
        cJSON_Delete(prev);
    }

    return out;
}

/** Add @p curr to @p root under @p name as full (dup) or diff; update the
 *  matching prev-cache. Empty sections are omitted (legacy behaviour). */
static void add_section(cJSON *root, const char *name, cJSON *curr,
                        bool full, char **prev_cache)
{
    if (curr == NULL)
    {
        return;
    }

    cJSON *section;

    if (full)
    {
        section = cJSON_Duplicate(curr, true);
    }
    else
    {
        section = diff_object(curr, *prev_cache);

        /* refresh the prev cache from the full current object */
        char *now = cJSON_PrintUnformatted(curr);

        if (now != NULL)
        {
            free(*prev_cache);
            *prev_cache = now; /* PSRAM heap via cJSON's default alloc */
        }
    }

    if (section != NULL && cJSON_GetArraySize(section) > 0)
    {
        cJSON_AddItemToObject(root, name, section);
    }
    else if (section != NULL)
    {
        cJSON_Delete(section); /* empty -> omit */
    }
}

static char *build_payload(bool full)
{
    cJSON *root = cJSON_CreateObject();

    if (root == NULL)
    {
        return NULL;
    }

    /* a fresh registration wants the complete state, not a diff against a
       previous HA's baseline */
    if (s_resync)
    {
        s_resync = false;
        drop_prev_caches();
        full = true;
    }

    cJSON_AddNumberToObject(root, "schema", HW_PAYLOAD_SCHEMA);

    /* status */
    cJSON *status = build_status();

    add_section(root, "status", status, full, &s_prev_status);

    /* contract v2: status.device_id / fw_version / hw_version are
       guaranteed in EVERY push (identity check + update entity). Diff
       mode drops unchanged keys, so overlay them from the fresh build. */
    cJSON *sent = cJSON_GetObjectItemCaseSensitive(root, "status");

    if (sent == NULL)
    {
        sent = cJSON_AddObjectToObject(root, "status");
    }

    if (sent != NULL && status != NULL)
    {
        static const char *IDENTITY_KEYS[] =
        { "device_id", "fw_version", "hw_version" };

        for (size_t i = 0;
             i < sizeof(IDENTITY_KEYS) / sizeof(IDENTITY_KEYS[0]); i++)
        {
            if (cJSON_GetObjectItemCaseSensitive(sent,
                                                 IDENTITY_KEYS[i]) != NULL)
            {
                continue;
            }

            const cJSON *v = cJSON_GetObjectItemCaseSensitive(
                status, IDENTITY_KEYS[i]);

            if (v != NULL)
            {
                cJSON_AddItemToObject(sent, IDENTITY_KEYS[i],
                                      cJSON_Duplicate(v, true));
            }
        }
    }

    cJSON_Delete(status);

    /* gps — always the FULL section when a fix exists (never diffed: the
       HA device tracker needs lat+lon together), omitted without a fix */
    cJSON *gps = build_gps();

    if (gps != NULL)
    {
        cJSON_AddItemToObject(root, "gps", gps);
    }

    char *body = cJSON_PrintUnformatted(root);

    cJSON_Delete(root);
    return body;
}

/* ---- gzip (ask #9 item 4: LTE data saver — JSON compresses 5-10x) ---------- */

/**
 * gzip @p in via the ROM miniz compressor (zero flash cost; the ~166 KB
 * compressor state is a lazy one-time PSRAM alloc, reused — the poster
 * is the only caller). Returns a PSRAM buffer (caller frees) or NULL —
 * the caller then falls back to the uncompressed body.
 */
static uint8_t *hw_gzip(const char *in, size_t in_len, size_t *out_len)
{
    static tdefl_compressor *s_comp;

    if (s_comp == NULL)
    {
        s_comp = heap_caps_malloc(sizeof(tdefl_compressor),
                                  MALLOC_CAP_SPIRAM);
    }

    if (s_comp == NULL || in_len == 0)
    {
        return NULL;
    }

    /* 10 B gzip header + raw deflate (worst case ~in_len + 5 B / 32 KB
       block + slack) + 8 B trailer (CRC32 + ISIZE) */
    size_t deflate_cap = in_len + in_len / 16 + 64;
    uint8_t *out = heap_caps_malloc(10 + deflate_cap + 8,
                                    MALLOC_CAP_SPIRAM);

    if (out == NULL)
    {
        return NULL;
    }

    static const uint8_t GZ_HDR[10] =
    { 0x1f, 0x8b, 8, 0, 0, 0, 0, 0, 0, 0xff }; /* deflate, no name, OS=unknown */

    memcpy(out, GZ_HDR, sizeof(GZ_HDR));

    /* no TDEFL_WRITE_ZLIB_HEADER -> raw deflate (what gzip framing wants) */
    if (tdefl_init(s_comp, NULL, NULL, TDEFL_DEFAULT_MAX_PROBES)
            != TDEFL_STATUS_OKAY)
    {
        free(out);
        return NULL;
    }

    size_t in_sz = in_len;
    size_t out_sz = deflate_cap;
    tdefl_status st = tdefl_compress(s_comp, in, &in_sz, out + 10,
                                     &out_sz, TDEFL_FINISH);

    if (st != TDEFL_STATUS_DONE || in_sz != in_len)
    {
        free(out);
        return NULL;
    }

    uint32_t crc = (uint32_t)mz_crc32(MZ_CRC32_INIT,
                                      (const unsigned char *)in, in_len);
    uint32_t isize = (uint32_t)in_len;
    uint8_t *tr = out + 10 + out_sz;

    for (int i = 0; i < 4; i++)
    {
        tr[i] = (uint8_t)(crc >> (8 * i));
        tr[4 + i] = (uint8_t)(isize >> (8 * i));
    }

    *out_len = 10 + out_sz + 8;
    return out;
}

/* ---- POST with failover --------------------------------------------------- */

static bool url_is_raw_ip_https(const char *url)
{
    if (strncasecmp(url, "https://", 8) != 0)
    {
        return false;
    }

    const char *h = url + 8;

    for (; *h && *h != '/' && *h != ':'; h++)
    {
        if ((*h < '0' || *h > '9') && *h != '.')
        {
            return false;
        }
    }

    return true;
}

/** Try each configured URL in order; first 2xx wins. Fills @p err;
 *  sets @p rejected on an HA identity rejection (HTTP 403). */
static bool post_failover(const hw_config_t *c, const void *body,
                          size_t body_len, bool gzipped,
                          char err[HW_ERR_LEN], char okstatus[HW_STATUS_LEN],
                          bool *rejected)
{
    static const char *const GZIP_HEADERS[] = { "Content-Encoding: gzip" };
    const char *urls[2] = { c->url, c->url2 };

    err[0] = '\0';
    *rejected = false;

    for (int i = 0; i < 2; i++)
    {
        if (urls[i][0] == '\0')
        {
            continue;
        }

        http_client_request_t req = {
            .url = urls[i],
            .method = HTTP_CLIENT_POST,
            .body = body,
            .body_len = body_len,
            .content_type = "application/json",
            .extra_headers = gzipped ? GZIP_HEADERS : NULL,
            .extra_header_count = gzipped ? 1 : 0,
            .cert_set = c->cert_set[0] != '\0' ? c->cert_set : NULL,
            .skip_common_name = url_is_raw_ip_https(urls[i]),
            .timeout_ms = HW_HTTP_TIMEOUT_MS,
        };
        http_client_response_t resp = { 0 };
        esp_err_t r = http_client_manager_request(&req, &resp);
        bool ok = (r == ESP_OK && resp.status_code >= 200 &&
                   resp.status_code < 300);

        if (ok)
        {
            snprintf(okstatus, HW_STATUS_LEN, "ok");
            http_client_manager_free(&resp);
            return true;
        }

        if (r == ESP_OK && resp.status_code == 403)
        {
            /* the push contract: 403 = our device_id does not match the
               configured device — the second URL is the SAME HA, so
               don't failover under a rejected identity */
            snprintf(err, HW_ERR_LEN, "http=403 identity rejected");
            *rejected = true;
            http_client_manager_free(&resp);
            return false;
        }

        if (r != ESP_OK)
        {
            snprintf(err, HW_ERR_LEN, "esp_err=%s", esp_err_to_name(r));
        }
        else
        {
            snprintf(err, HW_ERR_LEN, "http=%d", resp.status_code);
        }

        http_client_manager_free(&resp);
    }

    return false;
}

/* ---- the task ------------------------------------------------------------- */

static void poster_task(void *arg)
{
    (void)arg;

    uint32_t last_attempt = 0;

    vTaskDelay(pdMS_TO_TICKS(5000)); /* let the system settle */

    while (s_run)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        hw_config_t c;

        if (!hw_config_get(&c) || !c.enabled || c.url[0] == '\0')
        {
            continue;
        }

        /* ESPNetLink uplink check: the primary WAN is the LTE modem (no
           dev_status bit); WiFi STA (the mask) is the secondary path. */
        lte_status_t lte;
        bool lte_up = (lte_upstream_pppos_get_status(&lte) == ESP_OK &&
                       lte.ppp_connected);

        if (!lte_up &&
            !dev_status_manager_any_set(DEV_STATUS_NETWORK_CONNECTED_MASK))
        {
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        if (s_reject_streak >= HW_REJECT_LIMIT)
        {
            continue; /* identity rejected — paused until re-registration */
        }

        uint32_t now = xTaskGetTickCount() / configTICK_RATE_HZ;

        if ((now - last_attempt) < c.interval_s)
        {
            continue;
        }

        /* schedule on ATTEMPT, not success — a down endpoint must not be
           retried every second (legacy lesson) */
        last_attempt = now;

        char *body = build_payload(c.data_mode_full);

        if (body == NULL)
        {
            /* not silent (ESPNetLink): a permanently-failing build would
               otherwise look exactly like "poster never ran" */
            ESP_LOGW(TAG, "payload build failed; skipping this cycle");
            continue;
        }

        ESP_LOGD(TAG, "posting %u bytes to %s", (unsigned)strlen(body),
                 c.url);

        /* gzip (optional, either data mode): fall back to plain on any
           compression failure — delivery beats savings */
        const void *send_buf = body;
        size_t send_len = strlen(body);
        uint8_t *gz = NULL;
        bool gzipped = false;

        if (c.gzip)
        {
            size_t gz_len = 0;

            gz = hw_gzip(body, send_len, &gz_len);

            if (gz != NULL)
            {
                send_buf = gz;
                send_len = gz_len;
                gzipped = true;
            }
            else
            {
                ESP_LOGW(TAG, "gzip failed; posting uncompressed");
            }
        }

        char err[HW_ERR_LEN] = "";
        char okstatus[HW_STATUS_LEN] = "";
        bool rejected = false;
        bool ok = post_failover(&c, send_buf, send_len, gzipped, err,
                                okstatus, &rejected);

        free(gz);
        free(body);

        /* update stats (cache only — no flash) */
        hw_stats_t st;

        hw_stats_get(&st);

        if (ok)
        {
            s_reject_streak = 0;
            st.success_count++;
            st.retries = 0;
            strlcpy(st.status, "ok", sizeof(st.status));
            hw_format_utc(st.last_post);
            st.last_error[0] = '\0';
            st.last_error_time[0] = '\0';
        }
        else
        {
            s_reject_streak = rejected ? s_reject_streak + 1 : 0;
            st.fail_count++;
            st.retries++;

            if (s_reject_streak >= HW_REJECT_LIMIT)
            {
                strlcpy(st.status, "rejected", sizeof(st.status));
                ESP_LOGE(TAG,
                         "HA rejected our device identity (HTTP 403) %u "
                         "times — pausing pushes until re-registration "
                         "(was this device replaced?)",
                         (unsigned)s_reject_streak);
            }
            else
            {
                strlcpy(st.status, "failed", sizeof(st.status));
            }

            strlcpy(st.last_error, err, sizeof(st.last_error));
            hw_format_utc(st.last_error_time);
            ESP_LOGW(TAG, "webhook POST failed: %s", err);
        }

        hw_stats_set(&st);
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t hw_poster_start(void)
{
    if (s_task != NULL)
    {
        return ESP_OK;
    }

    s_run = true;
    s_task = xTaskCreateStatic(poster_task, "ha_webhook",
                               HW_STACK_WORDS, NULL, 4, s_stack, &s_tcb);
    return s_task != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

void hw_poster_stop(void)
{
    s_run = false; /* the task exits on its next lap */
}

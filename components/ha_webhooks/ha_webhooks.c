/**
 * @file ha_webhooks.c
 * @brief Lifecycle and the live config/stats caches (mutex-guarded,
 *        PSRAM). Settings schema + on_apply live in ha_webhooks_settings.c
 *        and store through hw_config_store. See ha_webhooks.h.
 */
#include "ha_webhooks.h"

#include <string.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "cJSON.h"
#include "log_manager.h"
#include "settings_manager.h"

#include "ha_webhooks_private.h"

static const char *TAG = "ha_webhooks";

static hw_config_t s_cfg EXT_RAM_BSS_ATTR;
static hw_stats_t  s_stats EXT_RAM_BSS_ATTR;
static bool        s_configured;

static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf;

static void lock(void)
{
    if (s_lock != NULL)
    {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (s_lock != NULL)
    {
        xSemaphoreGive(s_lock);
    }
}

void hw_format_utc(char out[HW_TS_LEN])
{
    time_t now = 0;

    out[0] = '\0';
    time(&now);

    struct tm utc;

    gmtime_r(&now, &utc);
    strftime(out, HW_TS_LEN, "%Y-%m-%dT%H:%M:%SZ", &utc);
}

/* ---- config / stats accessors --------------------------------------------- */

bool hw_config_get(hw_config_t *out)
{
    if (out == NULL || !s_configured)
    {
        return false;
    }

    lock();
    *out = s_cfg;
    unlock();
    return true;
}

void hw_stats_get(hw_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    lock();
    *out = s_stats;
    unlock();
}

void hw_stats_set(const hw_stats_t *in)
{
    if (in == NULL)
    {
        return;
    }

    lock();
    s_stats = *in;
    unlock();
}

void hw_config_store(const hw_config_t *cfg)
{
    lock();
    s_cfg = *cfg;
    unlock();
    s_configured = true;
}

/* ---- settings glue -------------------------------------------------------- */

/** Build the persist doc for a URL push. The push OWNS url/url2/enabled/
 *  interval/manual_override; the user-owned fields (data_mode, cert_set,
 *  cli) are preserved from the CURRENTLY STORED settings so a discovery
 *  push never clobbers a user's /api/settings choice (settings_manager_set
 *  is a full replace). */
static cJSON *config_to_json(const hw_config_t *c)
{
    cJSON *o = cJSON_CreateObject();

    if (o == NULL)
    {
        return NULL;
    }

    /* defaults if nothing stored yet */
    const char *data_mode = c->data_mode_full ? "full" : "changed";
    const char *cert_set = c->cert_set;
    bool gzip = c->gzip;
    bool cli = true;
    cJSON *cur = NULL;

    if (settings_manager_get("ha_webhooks", &cur) == ESP_OK && cur != NULL)
    {
        const cJSON *dm = cJSON_GetObjectItemCaseSensitive(cur, "data_mode");
        const cJSON *cs = cJSON_GetObjectItemCaseSensitive(cur, "cert_set");
        const cJSON *gz = cJSON_GetObjectItemCaseSensitive(cur, "gzip");
        const cJSON *cl = cJSON_GetObjectItemCaseSensitive(cur, "cli");

        if (cJSON_IsString(dm) && dm->valuestring != NULL)
        {
            data_mode = dm->valuestring;
        }

        if (cJSON_IsString(cs) && cs->valuestring != NULL)
        {
            cert_set = cs->valuestring;
        }

        gzip = cJSON_IsTrue(gz);
        cli = cl != NULL ? cJSON_IsTrue(cl) : true;
    }

    /* push-owned */
    cJSON_AddBoolToObject(o, "enabled", c->enabled);
    cJSON_AddStringToObject(o, "url", c->url);
    cJSON_AddStringToObject(o, "url2", c->url2);
    cJSON_AddNumberToObject(o, "interval_s", (double)c->interval_s);
    cJSON_AddBoolToObject(o, "manual_override", c->manual_override);
    /* preserved user settings */
    cJSON_AddStringToObject(o, "data_mode", data_mode);
    cJSON_AddStringToObject(o, "cert_set", cert_set);
    cJSON_AddBoolToObject(o, "gzip", gzip);
    cJSON_AddBoolToObject(o, "cli", cli);

    if (cur != NULL)
    {
        cJSON_Delete(cur);
    }

    return o;
}

esp_err_t hw_config_apply_live(const hw_config_t *cfg)
{
    if (cfg == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *doc = config_to_json(cfg);

    if (doc == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    /* persist (survives reboot) — runs on the httpd task (internal stack,
       flash-safe). Reboot-to-apply is deliberately bypassed for the URL
       push: also update the live cache so the poster picks it up now. */
    char err[96] = "";
    bool changed = false;
    esp_err_t r = settings_manager_set("ha_webhooks", doc, err,
                                       sizeof(err), &changed);

    cJSON_Delete(doc);

    if (r != ESP_OK)
    {
        ESP_LOGW(TAG, "persist failed: %s", err);
        return r;
    }

    hw_config_store(cfg);

    hw_poster_resync(); /* next post = full snapshot for the new HA */
    return ESP_OK;
}

/* ---- lifecycle ------------------------------------------------------------ */

esp_err_t ha_webhooks_init(void)
{
    static const log_descriptor_t LOG_DESC = { "ha_webhooks", ESP_LOG_INFO };

    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    strlcpy(s_stats.status, "disabled", sizeof(s_stats.status));

    log_manager_register(&LOG_DESC);
    return hw_settings_register();
}

esp_err_t ha_webhooks_start(void)
{
    if (!s_configured)
    {
        return ESP_ERR_INVALID_STATE; /* §4.3 step 5 */
    }

    hw_config_t c;

    (void)hw_config_get(&c);

    /* Always start the poster; its loop gates on `enabled` each lap. This
       way a LIVE enable via the HA discovery push (POST /api/webhook)
       takes effect without a reboot — the whole point of the endpoint. */
    ESP_LOGI(TAG, "up (enabled=%d, interval %us, mode %s)", c.enabled,
             (unsigned)c.interval_s, c.data_mode_full ? "full" : "changed");
    return hw_poster_start();
}

esp_err_t ha_webhooks_stop(void)
{
    hw_poster_stop();
    return ESP_OK;
}

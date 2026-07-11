/**
 * @file test_settings_main.c
 * @brief On-target test app for settings_manager: validation, persistence,
 *        CRC rollback, reboot-to-apply semantics, migration, and the boot
 *        fallback path. Drives clear markers for pytest to assert.
 */
#include <stdio.h>

#include "cJSON.h"
#include "esp_log.h"
#include "settings_manager.h"

static const char *TAG = "sm_test";

static const char *SCHEMA_V1 =
"{"
  "\"type\":\"object\","
  "\"properties\":{"
    "\"mode\":   {\"type\":\"string\",\"enum\":[\"disable\",\"sta\",\"ap\"],\"default\":\"sta\"},"
    "\"channel\":{\"type\":\"integer\",\"minimum\":1,\"maximum\":13,\"default\":6}"
  "}"
"}";   /* defaults live in the schema; no defaults_json override needed */

/* v2 renames "channel" -> "wifi_channel" to exercise on_migrate. */
static const char *SCHEMA_V2 =
"{"
  "\"type\":\"object\","
  "\"properties\":{"
    "\"mode\":{\"type\":\"string\",\"enum\":[\"disable\",\"sta\",\"ap\"],"
              "\"default\":\"sta\"},"
    "\"wifi_channel\":{\"type\":\"integer\",\"minimum\":1,\"maximum\":13,\"default\":6}"
  "}"
"}";

static int s_apply_count;

static esp_err_t demo_on_apply(const cJSON *s)
{
    const cJSON *ch = cJSON_GetObjectItemCaseSensitive(s, "channel");

    if (ch == NULL)
    {
        ch = cJSON_GetObjectItemCaseSensitive(s, "wifi_channel");
    }

    s_apply_count++;
    printf("APPLY count=%d channel=%d\n", s_apply_count,
           cJSON_IsNumber(ch) ? (int)ch->valuedouble : -1);
    return ESP_OK;
}

static esp_err_t demo_on_migrate(uint32_t from_version, cJSON *settings)
{
    printf("MIGRATE from=v%u\n", (unsigned)from_version);

    if (from_version == 1)
    {
        cJSON *ch = cJSON_DetachItemFromObjectCaseSensitive(settings, "channel");

        if (ch != NULL)
        {
            cJSON_AddItemToObject(settings, "wifi_channel", ch);
        }

        return ESP_OK;
    }

    return ESP_FAIL;
}

/* A second component whose on_apply always fails: exercises the §4.3 fallback
   (defaults retried, then left unconfigured + degraded). */
static esp_err_t broken_on_apply(const cJSON *s)
{
    (void)s;
    return ESP_FAIL;
}

static void register_demo(uint32_t version)
{
    settings_descriptor_t d =
    {
        .name          = "demo",
        .version       = version,
        .schema        = (version >= 2) ? SCHEMA_V2 : SCHEMA_V1,
        .defaults_json = NULL,          /* defaults come from the schema */
        .on_apply      = demo_on_apply,
        .on_migrate    = (version >= 2) ? demo_on_migrate : NULL,
    };

    ESP_ERROR_CHECK(settings_manager_register(&d));
}

static void set_channel(int ch)
{
    char   err[128] = {0};
    bool   changed  = false;
    cJSON *in       = cJSON_CreateObject();

    cJSON_AddNumberToObject(in, "channel", ch);
    esp_err_t r = settings_manager_set("demo", in, err, sizeof(err), &changed);
    cJSON_Delete(in);

    if (r == ESP_OK)
    {
        printf("SET ch=%d OK changed=%d\n", ch, (int)changed);
    }
    else
    {
        printf("SET ch=%d REJECT: %s\n", ch, err);
    }
}

static void print_current(const char *key)
{
    cJSON *cur = NULL;

    if (settings_manager_get("demo", &cur) == ESP_OK)
    {
        const cJSON *ch = cJSON_GetObjectItemCaseSensitive(cur, key);
        printf("CURRENT %s=%d\n", key,
               cJSON_IsNumber(ch) ? (int)ch->valuedouble : -1);
        cJSON_Delete(cur);
    }
}

/* Tear down and bring back up: simulates a reboot re-reading from disk. */
static void reboot_manager(uint32_t demo_version)
{
    settings_manager_stop();
    ESP_ERROR_CHECK(settings_manager_init());
    register_demo(demo_version);
    ESP_ERROR_CHECK(settings_manager_start());
}

/* Corrupt the persisted file by flipping a byte, then simulate a reboot. */
static void corrupt_and_reload(void)
{
    FILE *f = fopen("/settings/cfg/demo.json", "r+b");

    if (f == NULL)
    {
        printf("CORRUPT open-fail\n");
        return;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, size / 2, SEEK_SET);
    int c = fgetc(f);
    fseek(f, size / 2, SEEK_SET);
    fputc(c ^ 0xFF, f);
    fclose(f);
    printf("CORRUPT done\n");

    reboot_manager(2);
}

void app_main(void)
{
    ESP_LOGI(TAG, "start");

    ESP_ERROR_CHECK(settings_manager_init());

    /* Registration hardening: unparseable schema, invalid name, and a
       non-object defaults_json must all be rejected loudly at the call site. */
    settings_descriptor_t bad =
    {
        .name = "badschema", .version = 1,
        .schema = "{not json", .on_apply = demo_on_apply,
    };
    printf("REG badschema %s\n",
           settings_manager_register(&bad) == ESP_ERR_INVALID_ARG ? "REJECTED" : "ACCEPTED");

    bad.name   = "Bad/Name";
    bad.schema = "{}";
    printf("REG badname %s\n",
           settings_manager_register(&bad) == ESP_ERR_INVALID_ARG ? "REJECTED" : "ACCEPTED");

    bad.name          = "baddefaults";
    bad.schema        = "{}";
    bad.defaults_json = "[1,2,3]";
    printf("REG baddefaults %s\n",
           settings_manager_register(&bad) == ESP_ERR_INVALID_ARG ? "REJECTED" : "ACCEPTED");

    register_demo(1);
    ESP_ERROR_CHECK(settings_manager_start());   /* first boot -> defaults */
    print_current("channel");                    /* expect channel=6, APPLY count=1 */

    set_channel(11);                             /* valid -> persisted, NOT applied */
    printf("APPLY-COUNT-AFTER-SET %d\n", s_apply_count);   /* expect 1: no live apply */
    print_current("channel");                    /* pending value: 11 */

    set_channel(11);                             /* identical -> write + reboot skip */

    set_channel(99);                             /* invalid -> rejected */
    print_current("channel");                    /* still 11 */

    /* "Reboot": the persisted value survives and is applied NOW. */
    reboot_manager(1);
    print_current("channel");                    /* expect 11, APPLY count=2 (fresh) */

    /* Migration: same stored v1 file, component now registers v2. */
    reboot_manager(2);
    print_current("wifi_channel");               /* expect 11 under the new key */
    printf("DEGRADED demo=%d\n", (int)settings_manager_is_degraded("demo"));

    corrupt_and_reload();                        /* bad CRC -> defaults */
    print_current("wifi_channel");               /* expect 6 (rollback) */

    /* Boot fallback: a component that rejects even defaults degrades but the
       manager still returns OK and the other component still applied. */
    settings_manager_stop();
    ESP_ERROR_CHECK(settings_manager_init());
    register_demo(2);

    settings_descriptor_t broken =
    {
        .name     = "broken",
        .version  = 1,
        .schema   = "{\"type\":\"object\",\"properties\":{}}",
        .on_apply = broken_on_apply,
    };
    ESP_ERROR_CHECK(settings_manager_register(&broken));
    ESP_ERROR_CHECK(settings_manager_start());   /* must NOT fail overall */
    printf("DEGRADED broken=%d\n", (int)settings_manager_is_degraded("broken"));
    printf("DEGRADED demo=%d\n", (int)settings_manager_is_degraded("demo"));

    printf("TEST DONE\n");
}

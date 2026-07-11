/**
 * @file test_registry.c
 * @brief Host unit tests for the registry/lifecycle logic (settings_manager.c)
 *        against the rev 2 contract: reboot-to-apply, defaults_json override,
 *        on_migrate, boot fallback, write dedup + changed flag.
 *        Storage is the in-memory stub (test_storage_stub.c).
 */
#include <string.h>

#include "unity.h"
#include "settings_manager.h"
#include "test_storage_stub.h"

/* ---- fixtures -------------------------------------------------------------- */

static const char *SCHEMA_V2 =
"{"
  "\"type\":\"object\","
  "\"properties\":{"
    "\"rate\":{\"type\":\"integer\",\"minimum\":1,\"maximum\":100,\"default\":50},"
    "\"tag\": {\"type\":\"string\",\"default\":\"x\"}"
  "},"
  "\"required\":[\"rate\"]"
"}";

static int      s_apply_count;
static int      s_apply_fail_first_n;   /* fail this many on_apply calls */
static cJSON   *s_last_applied;         /* deep copy of last applied object */
static int      s_migrate_count;
static uint32_t s_migrate_from;
static bool     s_migrate_fail;
static bool     s_migrate_break_schema;

static esp_err_t spy_on_apply(const cJSON *settings)
{
    s_apply_count++;
    cJSON_Delete(s_last_applied);
    s_last_applied = cJSON_Duplicate(settings, true);

    if (s_apply_fail_first_n > 0)
    {
        s_apply_fail_first_n--;
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t spy_on_migrate(uint32_t from_version, cJSON *settings)
{
    s_migrate_count++;
    s_migrate_from = from_version;

    if (s_migrate_fail)
    {
        return ESP_FAIL;
    }

    /* v1 stored "speed"; v2 calls it "rate". */
    cJSON *speed = cJSON_DetachItemFromObjectCaseSensitive(settings, "speed");

    if (speed != NULL)
    {
        cJSON_AddItemToObject(settings, "rate", speed);
    }

    if (s_migrate_break_schema)
    {
        cJSON_DeleteItemFromObjectCaseSensitive(settings, "rate");
        cJSON_AddStringToObject(settings, "rate", "not-an-integer");
    }

    return ESP_OK;
}

static void fixture_reset(void)
{
    sm_stub_reset();
    s_apply_count          = 0;
    s_apply_fail_first_n   = 0;
    s_migrate_count        = 0;
    s_migrate_from         = 0;
    s_migrate_fail         = false;
    s_migrate_break_schema = false;
    cJSON_Delete(s_last_applied);
    s_last_applied = NULL;

    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
}

static void register_comp(uint32_t version, const char *defaults_json,
                          bool with_migrate)
{
    settings_descriptor_t d =
    {
        .name          = "comp",
        .version       = version,
        .schema        = SCHEMA_V2,
        .defaults_json = defaults_json,
        .on_apply      = spy_on_apply,
        .on_migrate    = with_migrate ? spy_on_migrate : NULL,
    };

    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_register(&d));
}

static int applied_rate(void)
{
    const cJSON *r = cJSON_GetObjectItemCaseSensitive(s_last_applied, "rate");
    return cJSON_IsNumber(r) ? (int)r->valuedouble : -1;
}

static esp_err_t set_rate(int rate, bool *changed)
{
    cJSON *in = cJSON_CreateObject();
    cJSON_AddNumberToObject(in, "rate", rate);
    esp_err_t r = settings_manager_set("comp", in, NULL, 0, changed);
    cJSON_Delete(in);
    return r;
}

/* ---- T1/T2: set() never applies; changed flag + write dedup ---------------- */

void test_set_never_applies(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(1, s_apply_count);              /* boot apply only */

    bool changed = false;
    TEST_ASSERT_EQUAL(ESP_OK, set_rate(75, &changed));
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL(1, s_apply_count);              /* STILL 1: no live apply */
}

void test_changed_flag_and_dedup(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    int saves_after_boot = sm_stub_save_count();
    bool changed = false;

    TEST_ASSERT_EQUAL(ESP_OK, set_rate(75, &changed));
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL(saves_after_boot + 1, sm_stub_save_count());

    TEST_ASSERT_EQUAL(ESP_OK, set_rate(75, &changed));  /* identical */
    TEST_ASSERT_FALSE(changed);
    TEST_ASSERT_EQUAL(saves_after_boot + 1, sm_stub_save_count()); /* no write */
}

/* ---- T3/T4: defaults ------------------------------------------------------- */

void test_defaults_json_whole_object_override(void)
{
    fixture_reset();
    register_comp(2, "{\"rate\":9,\"tag\":\"override\"}", false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL(9, applied_rate());              /* override, not schema 50 */

    cJSON *cur = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_get("comp", &cur));
    const cJSON *tag = cJSON_GetObjectItemCaseSensitive(cur, "tag");
    TEST_ASSERT_EQUAL_STRING("override", cJSON_GetStringValue(tag));
    cJSON_Delete(cur);
}

void test_schema_defaults_primary(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(50, applied_rate());             /* schema "default" */
}

/* ---- T5-T8: migration ------------------------------------------------------ */

void test_migrate_happy_path(void)
{
    fixture_reset();
    sm_stub_put("comp", 1, "{\"speed\":42}");          /* v1 shape */
    register_comp(2, NULL, true);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL(1, s_migrate_count);
    TEST_ASSERT_EQUAL(1, s_migrate_from);
    TEST_ASSERT_EQUAL(42, applied_rate());             /* value carried over */

    uint32_t ver = 0;
    cJSON *stored = sm_stub_read("comp", &ver);
    TEST_ASSERT_NOT_NULL(stored);
    TEST_ASSERT_EQUAL(2, ver);                         /* persisted at new ver */
    cJSON_Delete(stored);
    TEST_ASSERT_FALSE(settings_manager_is_degraded("comp"));
}

void test_migrate_failure_falls_back_to_defaults(void)
{
    fixture_reset();
    sm_stub_put("comp", 1, "{\"speed\":42}");
    s_migrate_fail = true;
    register_comp(2, NULL, true);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(50, applied_rate());             /* defaults, not 42 */
}

void test_migrate_invalid_result_falls_back(void)
{
    fixture_reset();
    sm_stub_put("comp", 1, "{\"speed\":42}");
    s_migrate_break_schema = true;
    register_comp(2, NULL, true);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(50, applied_rate());             /* schema rejected it */
}

void test_old_version_without_migrate_falls_back(void)
{
    fixture_reset();
    sm_stub_put("comp", 1, "{\"speed\":42}");
    register_comp(2, NULL, false);                     /* no on_migrate */
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(50, applied_rate());
}

/* ---- T9/T10: boot fallback ------------------------------------------------- */

void test_apply_failure_retries_with_defaults(void)
{
    fixture_reset();
    sm_stub_put("comp", 2, "{\"rate\":13}");           /* valid but "rejected" */
    s_apply_fail_first_n = 1;                          /* fail the first apply */
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL(2, s_apply_count);               /* stored, then defaults */
    TEST_ASSERT_EQUAL(50, applied_rate());             /* second call = defaults */
    TEST_ASSERT_TRUE(settings_manager_is_degraded("comp"));

    uint32_t ver = 0;
    cJSON *stored = sm_stub_read("comp", &ver);        /* defaults persisted */
    const cJSON *r = cJSON_GetObjectItemCaseSensitive(stored, "rate");
    TEST_ASSERT_EQUAL(50, (int)r->valuedouble);
    cJSON_Delete(stored);
}

static int s_sibling_applied;

/* Always-OK apply for the sibling component, independent of the failure spy. */
static esp_err_t sibling_apply(const cJSON *s)
{
    (void)s;
    s_sibling_applied++;
    return ESP_OK;
}

void test_defaults_also_fail_leaves_unconfigured(void)
{
    fixture_reset();
    s_sibling_applied    = 0;
    s_apply_fail_first_n = 99;                         /* everything fails */
    register_comp(2, NULL, false);

    settings_descriptor_t ok =
    {
        .name     = "okcomp",
        .version  = 1,
        .schema   = "{\"type\":\"object\",\"properties\":{}}",
        .on_apply = sibling_apply,
    };
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_register(&ok));

    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start()); /* boot survives */
    TEST_ASSERT_TRUE(settings_manager_is_degraded("comp"));
    TEST_ASSERT_FALSE(settings_manager_is_degraded("okcomp"));
    TEST_ASSERT_EQUAL(1, s_sibling_applied);           /* sibling still applied */
}

/* ---- T11/T12: registration hardening --------------------------------------- */

void test_register_rejects_bad_defaults_json(void)
{
    fixture_reset();

    settings_descriptor_t d =
    {
        .name          = "comp",
        .version       = 1,
        .schema        = SCHEMA_V2,
        .defaults_json = "[1,2,3]",                    /* not an object */
        .on_apply      = spy_on_apply,
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, settings_manager_register(&d));

    d.defaults_json = "{broken";                       /* not JSON at all */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, settings_manager_register(&d));
}

void test_register_after_start_rejected(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    settings_descriptor_t d =
    {
        .name     = "late",
        .version  = 1,
        .schema   = "{}",
        .on_apply = spy_on_apply,
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, settings_manager_register(&d));
}

/* ---- full-replace semantics ------------------------------------------------ */

void test_set_is_full_replace_not_merge(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    /* Establish a non-default tag. */
    cJSON *in = cJSON_CreateObject();
    cJSON_AddNumberToObject(in, "rate", 75);
    cJSON_AddStringToObject(in, "tag", "custom");
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_set("comp", in, NULL, 0, NULL));
    cJSON_Delete(in);

    /* Full replace: omitting "tag" must reset it to the schema default,
       NOT keep "custom" (which merge semantics would). */
    bool changed = false;
    TEST_ASSERT_EQUAL(ESP_OK, set_rate(75, &changed));
    TEST_ASSERT_TRUE(changed);

    cJSON *cur = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_get("comp", &cur));
    const cJSON *tag = cJSON_GetObjectItemCaseSensitive(cur, "tag");
    TEST_ASSERT_EQUAL_STRING("x", cJSON_GetStringValue(tag));
    cJSON_Delete(cur);
}

/* ---- backup / restore (settings_manager_backup.c) -------------------------- */

void test_export_shape_and_versions(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    cJSON *in = cJSON_Parse("{\"rate\":7}");

    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_set("comp", in, NULL, 0, NULL));
    cJSON_Delete(in);

    cJSON *out = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_export(&out));

    const cJSON *comp = cJSON_GetObjectItemCaseSensitive(out, "comp");
    const cJSON *ver  = cJSON_GetObjectItemCaseSensitive(comp, "version");
    const cJSON *data = cJSON_GetObjectItemCaseSensitive(comp, "data");
    const cJSON *rate = cJSON_GetObjectItemCaseSensitive(data, "rate");
    const cJSON *tag  = cJSON_GetObjectItemCaseSensitive(data, "tag");

    TEST_ASSERT_NOT_NULL(comp);
    TEST_ASSERT_EQUAL(2, (int)ver->valuedouble);
    TEST_ASSERT_EQUAL(7, rate->valueint);
    /* set() filled the omitted key from schema defaults -> exported too */
    TEST_ASSERT_EQUAL_STRING("x", tag->valuestring);
    cJSON_Delete(out);
}

void test_restore_same_version_persists(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    cJSON *in = cJSON_Parse("{\"rate\":9,\"tag\":\"y\"}");
    bool changed = false;

    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_restore("comp", 2, in, false, NULL, 0,
                                               &changed));
    cJSON_Delete(in);
    TEST_ASSERT_TRUE(changed);

    cJSON *stored = sm_stub_read("comp", NULL);

    TEST_ASSERT_EQUAL(9, cJSON_GetObjectItemCaseSensitive(stored,
                                                          "rate")->valueint);
    cJSON_Delete(stored);
}

void test_restore_migrates_older_backup(void)
{
    fixture_reset();
    register_comp(2, NULL, true);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    /* v1 backups say "speed"; spy_on_migrate renames it to "rate" */
    cJSON *in = cJSON_Parse("{\"speed\":33}");
    bool changed = false;

    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_restore("comp", 1, in, false, NULL, 0,
                                               &changed));
    cJSON_Delete(in);
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL(1, s_migrate_count);
    TEST_ASSERT_EQUAL(1, (int)s_migrate_from);

    uint32_t ver = 0;
    cJSON *stored = sm_stub_read("comp", &ver);

    /* persisted at the CURRENT schema version, migrated shape */
    TEST_ASSERT_EQUAL(2, (int)ver);
    TEST_ASSERT_EQUAL(33, cJSON_GetObjectItemCaseSensitive(stored,
                                                           "rate")->valueint);
    cJSON_Delete(stored);
}

void test_restore_newer_backup_rejected(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    cJSON *in = cJSON_Parse("{\"rate\":9}");
    char err[96] = "";

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_VERSION,
                      settings_manager_restore("comp", 3, in, false, err,
                                               sizeof(err), NULL));
    cJSON_Delete(in);
    TEST_ASSERT_NOT_NULL(strstr(err, "update the firmware"));
}

void test_restore_dry_run_validates_but_persists_nothing(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    int saves = sm_stub_save_count();

    /* valid data: dry run OK, nothing written */
    cJSON *ok = cJSON_Parse("{\"rate\":9}");
    bool changed = true;

    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_restore("comp", 2, ok, true, NULL, 0,
                                               &changed));
    cJSON_Delete(ok);
    TEST_ASSERT_FALSE(changed);
    TEST_ASSERT_EQUAL(saves, sm_stub_save_count());

    /* invalid data: dry run reports the validator's message */
    cJSON *bad = cJSON_Parse("{\"rate\":0}");
    char err[96] = "";

    TEST_ASSERT_NOT_EQUAL(ESP_OK,
                          settings_manager_restore("comp", 2, bad, true, err,
                                                   sizeof(err), NULL));
    cJSON_Delete(bad);
    TEST_ASSERT_TRUE(err[0] != '\0');
    TEST_ASSERT_EQUAL(saves, sm_stub_save_count());
}

void test_restore_unknown_component_not_found(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    cJSON *in = cJSON_Parse("{\"rate\":9}");

    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND,
                      settings_manager_restore("nope", 1, in, false, NULL, 0,
                                               NULL));
    cJSON_Delete(in);
}

void test_restore_identical_reports_unchanged(void)
{
    fixture_reset();
    register_comp(2, NULL, false);
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    cJSON *in = cJSON_Parse("{\"rate\":9,\"tag\":\"y\"}");
    bool changed = false;

    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_restore("comp", 2, in, false, NULL, 0,
                                               &changed));
    TEST_ASSERT_TRUE(changed);

    /* the exact same object again: write-dedup -> changed=false */
    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_restore("comp", 2, in, false, NULL, 0,
                                               &changed));
    cJSON_Delete(in);
    TEST_ASSERT_FALSE(changed);
}

/* ---- pending_reboot (applied-at-boot snapshot) ------------------------------ */

void test_pending_reboot_lifecycle(void)
{
    fixture_reset();
    register_comp(2, NULL, false);

    /* before start: nothing applied yet -> false */
    TEST_ASSERT_FALSE(settings_manager_is_pending_reboot("comp"));

    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_FALSE(settings_manager_is_pending_reboot("comp"));

    /* a persisted change diverges from the boot-applied snapshot */
    cJSON *in = cJSON_Parse("{\"rate\":9}");

    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_set("comp", in, NULL, 0, NULL));
    cJSON_Delete(in);
    TEST_ASSERT_TRUE(settings_manager_is_pending_reboot("comp"));

    /* saving the boot-applied values back clears it (exact compare) */
    cJSON *back = cJSON_Parse("{\"rate\":50,\"tag\":\"x\"}"); /* the defaults */

    TEST_ASSERT_EQUAL(ESP_OK,
                      settings_manager_set("comp", back, NULL, 0, NULL));
    cJSON_Delete(back);
    TEST_ASSERT_FALSE(settings_manager_is_pending_reboot("comp"));

    /* unknown component -> false, never an error */
    TEST_ASSERT_FALSE(settings_manager_is_pending_reboot("nope"));
}

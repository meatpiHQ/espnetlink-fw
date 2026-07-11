/**
 * @file test_main.c
 * @brief Host tests for api_http's pure helpers (api_http_util.c): secret
 *        redaction on GET, keep-stored unredaction on PUT, settings URI
 *        parsing, and the log-level name map.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "cJSON.h"
#include "esp_log.h"

#include "api_http_private.h"

/* GET path: every string whose key ends in a secret suffix is blanked;
 * non-secret keys and non-string secrets stay untouched. */
void test_redact_blanks_secret_strings_only(void)
{
    cJSON *obj = cJSON_Parse("{"
                             "\"apn_password\":\"s3cret\","
                             "\"wifi_private_key\":\"pem\","
                             "\"APN_PIN\":\"1234\","
                             "\"apn\":\"internet\","
                             "\"enabled\":true"
                             "}");

    TEST_ASSERT_NOT_NULL(obj);
    api_util_redact(obj);

    TEST_ASSERT_EQUAL_STRING("", cJSON_GetObjectItemCaseSensitive(
                                     obj, "apn_password")->valuestring);
    TEST_ASSERT_EQUAL_STRING("", cJSON_GetObjectItemCaseSensitive(
                                     obj, "wifi_private_key")->valuestring);
    TEST_ASSERT_EQUAL_STRING("", cJSON_GetObjectItemCaseSensitive(
                                     obj, "APN_PIN")->valuestring);
    TEST_ASSERT_EQUAL_STRING("internet", cJSON_GetObjectItemCaseSensitive(
                                             obj, "apn")->valuestring);
    TEST_ASSERT_TRUE(cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(
                                      obj, "enabled")));
    cJSON_Delete(obj);
}

/* PUT path: an empty secret means "keep the stored value"; a non-empty one
 * replaces it; empty NON-secret fields are honored verbatim. */
void test_unredact_restores_kept_secrets(void)
{
    cJSON *in = cJSON_Parse("{"
                            "\"apn_password\":\"\","
                            "\"apn_pin_auth_key\":\"newkey\","
                            "\"apn\":\"\""
                            "}");
    cJSON *stored = cJSON_Parse("{"
                                "\"apn_password\":\"keepme\","
                                "\"apn_pin_auth_key\":\"oldkey\","
                                "\"apn\":\"internet\""
                                "}");

    TEST_ASSERT_NOT_NULL(in);
    TEST_ASSERT_NOT_NULL(stored);
    api_util_unredact(in, stored);

    TEST_ASSERT_EQUAL_STRING("keepme", cJSON_GetObjectItemCaseSensitive(
                                           in, "apn_password")->valuestring);
    TEST_ASSERT_EQUAL_STRING("newkey", cJSON_GetObjectItemCaseSensitive(
                                           in, "apn_pin_auth_key")
                                           ->valuestring);
    /* empty NON-secret string must stay empty */
    TEST_ASSERT_EQUAL_STRING("", cJSON_GetObjectItemCaseSensitive(
                                     in, "apn")->valuestring);
    cJSON_Delete(in);
    cJSON_Delete(stored);
}

/* An empty secret with NO stored counterpart stays empty (fresh device). */
void test_unredact_without_stored_value_keeps_empty(void)
{
    cJSON *in = cJSON_Parse("{\"apn_password\":\"\"}");
    cJSON *stored = cJSON_Parse("{}");

    TEST_ASSERT_NOT_NULL(in);
    TEST_ASSERT_NOT_NULL(stored);
    api_util_unredact(in, stored);

    TEST_ASSERT_EQUAL_STRING("", cJSON_GetObjectItemCaseSensitive(
                                     in, "apn_password")->valuestring);
    cJSON_Delete(in);
    cJSON_Delete(stored);
}

/* URI parser: name and /schema split, rejections for empty names, extra
 * segments, wrong prefix, and names that overflow the buffer. */
void test_settings_path_parses_name_and_schema(void)
{
    char name[32];
    bool is_schema = true;

    TEST_ASSERT_EQUAL(ESP_OK, api_util_settings_path(
                                  "/api/settings/gps", name, sizeof(name),
                                  &is_schema));
    TEST_ASSERT_EQUAL_STRING("gps", name);
    TEST_ASSERT_FALSE(is_schema);

    TEST_ASSERT_EQUAL(ESP_OK, api_util_settings_path(
                                  "/api/settings/lte_upstream_pppos/schema",
                                  name, sizeof(name), &is_schema));
    TEST_ASSERT_EQUAL_STRING("lte_upstream_pppos", name);
    TEST_ASSERT_TRUE(is_schema);
}

void test_settings_path_rejects_bad_uris(void)
{
    char name[8];
    bool is_schema = false;

    /* empty name */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, api_util_settings_path(
                          "/api/settings/", name, sizeof(name), &is_schema));
    /* extra path segment */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, api_util_settings_path(
                          "/api/settings/a/b", name, sizeof(name),
                          &is_schema));
    /* wrong prefix */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, api_util_settings_path(
                          "/api/other/gps", name, sizeof(name), &is_schema));
    /* name longer than the 8-byte buffer */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, api_util_settings_path(
                          "/api/settings/waytoolongname", name, sizeof(name),
                          &is_schema));
    /* NULL args */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, api_util_settings_path(
                          NULL, name, sizeof(name), &is_schema));
}

/* Log-level map: every published name resolves, junk doesn't. */
void test_level_from_str_maps_all_names(void)
{
    int level = -1;

    TEST_ASSERT_EQUAL(ESP_OK, api_util_level_from_str("none", &level));
    TEST_ASSERT_EQUAL(ESP_LOG_NONE, level);
    TEST_ASSERT_EQUAL(ESP_OK, api_util_level_from_str("error", &level));
    TEST_ASSERT_EQUAL(ESP_LOG_ERROR, level);
    TEST_ASSERT_EQUAL(ESP_OK, api_util_level_from_str("warn", &level));
    TEST_ASSERT_EQUAL(ESP_LOG_WARN, level);
    TEST_ASSERT_EQUAL(ESP_OK, api_util_level_from_str("info", &level));
    TEST_ASSERT_EQUAL(ESP_LOG_INFO, level);
    TEST_ASSERT_EQUAL(ESP_OK, api_util_level_from_str("debug", &level));
    TEST_ASSERT_EQUAL(ESP_LOG_DEBUG, level);
    TEST_ASSERT_EQUAL(ESP_OK, api_util_level_from_str("verbose", &level));
    TEST_ASSERT_EQUAL(ESP_LOG_VERBOSE, level);

    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND,
                      api_util_level_from_str("loud", &level));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      api_util_level_from_str(NULL, &level));
}

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_redact_blanks_secret_strings_only);
    RUN_TEST(test_unredact_restores_kept_secrets);
    RUN_TEST(test_unredact_without_stored_value_keeps_empty);
    RUN_TEST(test_settings_path_parses_name_and_schema);
    RUN_TEST(test_settings_path_rejects_bad_uris);
    RUN_TEST(test_level_from_str_maps_all_names);

    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}

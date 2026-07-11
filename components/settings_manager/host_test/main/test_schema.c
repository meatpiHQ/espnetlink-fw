/**
 * @file test_schema.c
 * @brief Host (linux target) unit tests for the settings schema validator.
 */
#include <string.h>

#include "unity.h"
#include "cJSON.h"
#include "settings_manager_private.h"

/* Schema covering every supported keyword, including the file extension. */
static const char *SCHEMA =
"{"
  "\"type\":\"object\","
  "\"properties\":{"
    "\"ssid\":   {\"type\":\"string\",\"minLength\":1,\"maxLength\":4},"
    "\"mode\":   {\"type\":\"string\",\"enum\":[\"disable\",\"sta\",\"ap\"]},"
    "\"channel\":{\"type\":\"integer\",\"minimum\":1,\"maximum\":13},"
    "\"power\":  {\"type\":\"number\",\"minimum\":0.0,\"maximum\":20.0},"
    "\"enabled\":{\"type\":\"boolean\"},"
    "\"ca_cert\":{\"type\":\"string\",\"format\":\"file\"}"
  "},"
  "\"required\":[\"mode\"]"
"}";

/* Stub: pretend only "present.pem" exists on the FS. */
static bool stub_file_exists(const char *name)
{
    return strcmp(name, "present.pem") == 0;
}

static esp_err_t validate(const char *json)
{
    cJSON *d = cJSON_Parse(json);
    char err[128] = {0};
    esp_err_t r = sm_schema_validate(SCHEMA, d, stub_file_exists, err, sizeof(err));
    cJSON_Delete(d);
    return r;
}

void test_valid_object_accepted(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, validate("{\"mode\":\"sta\",\"channel\":6}"));
}

void test_missing_required_rejected(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"channel\":6}"));
}

void test_wrong_type_rejected(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"mode\":\"sta\",\"channel\":\"x\"}"));
}

void test_integer_out_of_range_rejected(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"mode\":\"sta\",\"channel\":99}"));
}

void test_non_integer_for_integer_rejected(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"mode\":\"sta\",\"channel\":6.5}"));
}

void test_number_in_range_accepted(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, validate("{\"mode\":\"ap\",\"power\":12.5}"));
}

void test_enum_member_accepted(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, validate("{\"mode\":\"disable\"}"));
}

void test_enum_non_member_rejected(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"mode\":\"bridge\"}"));
}

void test_string_too_long_rejected(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"mode\":\"sta\",\"ssid\":\"toolong\"}"));
}

void test_boolean_type_enforced(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"mode\":\"sta\",\"enabled\":\"yes\"}"));
}

void test_file_ref_present_accepted(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, validate("{\"mode\":\"sta\",\"ca_cert\":\"present.pem\"}"));
}

void test_file_ref_missing_rejected(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate("{\"mode\":\"sta\",\"ca_cert\":\"gone.pem\"}"));
}

void test_unknown_key_allowed(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, validate("{\"mode\":\"sta\",\"extra\":123}"));
}

void test_collect_defaults(void)
{
    const char *s =
    "{\"type\":\"object\",\"properties\":{"
      "\"mode\":{\"type\":\"string\",\"default\":\"sta\"},"
      "\"channel\":{\"type\":\"integer\",\"default\":6},"
      "\"ssid\":{\"type\":\"string\"}}}";   /* ssid has no default */

    cJSON *defs = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_collect_defaults(s, &defs));
    TEST_ASSERT_NOT_NULL(defs);

    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(defs, "mode");
    const cJSON *chan = cJSON_GetObjectItemCaseSensitive(defs, "channel");
    TEST_ASSERT_NOT_NULL(mode);
    TEST_ASSERT_EQUAL_STRING("sta", mode->valuestring);
    TEST_ASSERT_EQUAL(6, (int)chan->valuedouble);
    TEST_ASSERT_NULL(cJSON_GetObjectItemCaseSensitive(defs, "ssid"));

    cJSON_Delete(defs);
}

/* ---- bounded arrays (added 2026-07-03 for socket_manager/bridge_manager) --- */

/* array of flat objects — the servers/bridges shape */
static const char *ARR_SCHEMA =
"{\"type\":\"object\",\"properties\":{"
  "\"servers\":{\"type\":\"array\",\"maxItems\":4,\"items\":{"
    "\"type\":\"object\",\"properties\":{"
      "\"port\":{\"type\":\"integer\",\"minimum\":1,\"maximum\":65535},"
      "\"proto\":{\"type\":\"string\",\"enum\":[\"tcp\",\"udp\"]}},"
    "\"required\":[\"port\"]}},"
  "\"tags\":{\"type\":\"array\",\"maxItems\":3,"
    "\"items\":{\"type\":\"string\",\"maxLength\":4}},"
  "\"bad\":{\"type\":\"array\",\"items\":{\"type\":\"integer\"}}"
"}}";

static esp_err_t validate_arr(const char *json, char *err, size_t err_len)
{
    cJSON *d = cJSON_Parse(json);
    esp_err_t r = sm_schema_validate(ARR_SCHEMA, d, NULL, err, err_len);
    cJSON_Delete(d);
    return r;
}

void test_array_of_objects_accepted(void)
{
    char err[96] = "";
    TEST_ASSERT_EQUAL(ESP_OK, validate_arr(
        "{\"servers\":[{\"port\":3333,\"proto\":\"tcp\"},{\"port\":17}]}",
        err, sizeof(err)));
}

void test_array_over_max_items_rejected(void)
{
    char err[96] = "";
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr(
        "{\"servers\":[{\"port\":1},{\"port\":2},{\"port\":3},"
        "{\"port\":4},{\"port\":5}]}", err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "maxItems"));
}

void test_array_item_bad_field_rejected_with_index(void)
{
    char err[96] = "";
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr(
        "{\"servers\":[{\"port\":3333},{\"port\":99999}]}",
        err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "servers[1]"));  /* index in message */
}

void test_array_item_missing_required_rejected(void)
{
    char err[96] = "";
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr(
        "{\"servers\":[{\"proto\":\"tcp\"}]}", err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "port"));
}

void test_array_item_enum_enforced(void)
{
    char err[96] = "";
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr(
        "{\"servers\":[{\"port\":1,\"proto\":\"tls\"}]}", err, sizeof(err)));
}

void test_scalar_array_items_checked(void)
{
    char err[96] = "";
    TEST_ASSERT_EQUAL(ESP_OK, validate_arr(
        "{\"tags\":[\"a\",\"bb\"]}", err, sizeof(err)));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr(
        "{\"tags\":[\"toolong\"]}", err, sizeof(err)));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr(
        "{\"tags\":[7]}", err, sizeof(err)));
}

void test_array_without_max_items_is_schema_error(void)
{
    char err[96] = "";
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr("{\"bad\":[1]}",
                                               err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "maxItems"));
}

void test_non_array_for_array_rejected(void)
{
    char err[96] = "";
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate_arr("{\"servers\":7}",
                                               err, sizeof(err)));
}

void test_array_default_collected(void)
{
    const char *s =
    "{\"type\":\"object\",\"properties\":{"
      "\"servers\":{\"type\":\"array\",\"maxItems\":4,"
        "\"default\":[{\"port\":35000}]}}}";
    cJSON *defs = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_collect_defaults(s, &defs));

    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(defs, "servers");
    TEST_ASSERT_TRUE(cJSON_IsArray(arr));
    TEST_ASSERT_EQUAL(1, cJSON_GetArraySize(arr));
    cJSON_Delete(defs);
}

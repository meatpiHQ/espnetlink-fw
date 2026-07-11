/**
 * @file test_fields.c
 * @brief Unit tests for the field-table -> JSON Schema generator
 *        (sm_schema_from_fields) and its round trip through the validator
 *        and defaults collector.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"
#include "cJSON.h"

#include "settings_manager.h"
#include "settings_manager_private.h"

/* clang-format off */
static const settings_field_t FIELDS[] =
{
    SETTINGS_STR_ENUM("mode",     "off,sta,ap,apsta", "apsta"),
    SETTINGS_STR     ("ssid",     32, ""),
    SETTINGS_STR_LEN ("password", 8, 64, "12345678"),
    SETTINGS_INT     ("channel",  1, 13, 6),
    SETTINGS_INT     ("retry",    -1, 1000, -1),
    SETTINGS_BOOL    ("auto",     true),
};
/* clang-format on */
#define FIELD_COUNT (sizeof(FIELDS) / sizeof(FIELDS[0]))

static char *build(void)
{
    char *schema = NULL;

    TEST_ASSERT_EQUAL(ESP_OK,
                      sm_schema_from_fields(FIELDS, FIELD_COUNT, &schema));
    TEST_ASSERT_NOT_NULL(schema);
    return schema;
}

void test_fields_generated_schema_parses_with_expected_keywords(void)
{
    char  *schema = build();
    cJSON *root = cJSON_Parse(schema);

    TEST_ASSERT_NOT_NULL(root);

    cJSON *props = cJSON_GetObjectItemCaseSensitive(root, "properties");
    cJSON *mode = cJSON_GetObjectItemCaseSensitive(props, "mode");
    cJSON *pass = cJSON_GetObjectItemCaseSensitive(props, "password");
    cJSON *chan = cJSON_GetObjectItemCaseSensitive(props, "channel");

    TEST_ASSERT_EQUAL_INT(4, cJSON_GetArraySize(
        cJSON_GetObjectItemCaseSensitive(mode, "enum")));
    TEST_ASSERT_EQUAL_INT(8, (int)cJSON_GetObjectItemCaseSensitive(
        pass, "minLength")->valuedouble);
    TEST_ASSERT_EQUAL_INT(64, (int)cJSON_GetObjectItemCaseSensitive(
        pass, "maxLength")->valuedouble);
    TEST_ASSERT_EQUAL_INT(13, (int)cJSON_GetObjectItemCaseSensitive(
        chan, "maximum")->valuedouble);

    cJSON_Delete(root);
    free(schema);
}

void test_fields_roundtrip_validator_accepts_and_rejects(void)
{
    char  *schema = build();
    cJSON *good = cJSON_Parse("{\"mode\":\"sta\",\"ssid\":\"x\","
                              "\"password\":\"12345678\",\"channel\":6,"
                              "\"retry\":-1,\"auto\":true}");
    cJSON *bad_enum = cJSON_Parse("{\"mode\":\"nope\"}");
    cJSON *bad_range = cJSON_Parse("{\"channel\":99}");
    char   err[96];

    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_validate(schema, good, NULL,
                                                 err, sizeof(err)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_validate(schema, bad_enum, NULL,
                                         err, sizeof(err)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_validate(schema, bad_range, NULL,
                                         err, sizeof(err)));

    cJSON_Delete(good);
    cJSON_Delete(bad_enum);
    cJSON_Delete(bad_range);
    free(schema);
}

void test_fields_defaults_collected(void)
{
    char  *schema = build();
    cJSON *defs = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_collect_defaults(schema, &defs));

    TEST_ASSERT_EQUAL_STRING("apsta", cJSON_GetObjectItemCaseSensitive(
        defs, "mode")->valuestring);
    TEST_ASSERT_EQUAL_INT(6, (int)cJSON_GetObjectItemCaseSensitive(
        defs, "channel")->valuedouble);
    TEST_ASSERT_TRUE(cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(
        defs, "auto")));
    TEST_ASSERT_EQUAL_INT((int)FIELD_COUNT, cJSON_GetArraySize(defs));

    cJSON_Delete(defs);
    free(schema);
}

void test_fields_rejects_duplicate_and_empty_keys(void)
{
    settings_field_t dup[] =
    {
        SETTINGS_STR("a", 8, ""),
        SETTINGS_STR("a", 8, ""),
    };
    settings_field_t empty[] =
    {
        SETTINGS_STR("", 8, ""),
    };
    char *out = NULL;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(dup, 2, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(empty, 1, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(NULL, 0, &out));
}

/* ---- arrays of objects (rev 2.5) --------------------------------------- */

/* clang-format off */
static const settings_field_t SERVER_ITEMS[] =
{
    SETTINGS_STR_REQ     ("name",  1, 15, "srv"),
    SETTINGS_STR_ENUM_REQ("proto", "tcp,udp", "tcp"),
    SETTINGS_INT_REQ     ("port",  1, 65535, 3333),
    SETTINGS_INT         ("keepalive_s", 0, 600, 0),
    SETTINGS_BOOL        ("enabled", false),
};

static const settings_field_t ARRAY_FIELDS[] =
{
    SETTINGS_BOOL ("enabled", true),
    SETTINGS_ARRAY("servers", 4, SERVER_ITEMS,
        SETTINGS_JSON([
            {"name":"obd0","proto":"tcp","port":35000,"enabled":true}
        ])),
    SETTINGS_ARRAY_ANY("rules", 16, NULL),
};
/* clang-format on */
#define ARRAY_FIELD_COUNT (sizeof(ARRAY_FIELDS) / sizeof(ARRAY_FIELDS[0]))

static char *build_array(void)
{
    char *schema = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_from_fields(ARRAY_FIELDS,
                                                    ARRAY_FIELD_COUNT,
                                                    &schema));
    TEST_ASSERT_NOT_NULL(schema);
    return schema;
}

void test_fields_array_schema_shape(void)
{
    char  *schema = build_array();
    cJSON *root = cJSON_Parse(schema);
    cJSON *props = cJSON_GetObjectItemCaseSensitive(root, "properties");
    cJSON *servers = cJSON_GetObjectItemCaseSensitive(props, "servers");
    cJSON *rules = cJSON_GetObjectItemCaseSensitive(props, "rules");

    TEST_ASSERT_EQUAL_STRING("array", cJSON_GetObjectItemCaseSensitive(
        servers, "type")->valuestring);
    TEST_ASSERT_EQUAL_INT(4, (int)cJSON_GetObjectItemCaseSensitive(
        servers, "maxItems")->valuedouble);

    cJSON *items = cJSON_GetObjectItemCaseSensitive(servers, "items");
    cJSON *iprops = cJSON_GetObjectItemCaseSensitive(items, "properties");
    cJSON *ireq = cJSON_GetObjectItemCaseSensitive(items, "required");

    TEST_ASSERT_EQUAL_STRING("object", cJSON_GetObjectItemCaseSensitive(
        items, "type")->valuestring);
    TEST_ASSERT_EQUAL_INT(5, cJSON_GetArraySize(iprops));
    TEST_ASSERT_EQUAL_INT(3, cJSON_GetArraySize(ireq));

    /* default carried through as a real array */
    cJSON *def = cJSON_GetObjectItemCaseSensitive(servers, "default");

    TEST_ASSERT_TRUE(cJSON_IsArray(def));
    TEST_ASSERT_EQUAL_INT(1, cJSON_GetArraySize(def));

    /* free-form items: object type, no properties/required; default [] */
    cJSON *ritems = cJSON_GetObjectItemCaseSensitive(rules, "items");

    TEST_ASSERT_EQUAL_STRING("object", cJSON_GetObjectItemCaseSensitive(
        ritems, "type")->valuestring);
    TEST_ASSERT_NULL(cJSON_GetObjectItemCaseSensitive(ritems, "properties"));
    TEST_ASSERT_TRUE(cJSON_IsArray(
        cJSON_GetObjectItemCaseSensitive(rules, "default")));
    TEST_ASSERT_EQUAL_INT(0, cJSON_GetArraySize(
        cJSON_GetObjectItemCaseSensitive(rules, "default")));

    cJSON_Delete(root);
    free(schema);
}

void test_fields_array_roundtrip_validator(void)
{
    char *schema = build_array();
    char  err[96];

    cJSON *good = cJSON_Parse(
        "{\"servers\":[{\"name\":\"a\",\"proto\":\"udp\",\"port\":17}],"
        "\"rules\":[{\"anything\":\"goes\"}]}");
    cJSON *missing_req = cJSON_Parse(
        "{\"servers\":[{\"name\":\"a\",\"proto\":\"tcp\"}]}");
    cJSON *bad_enum = cJSON_Parse(
        "{\"servers\":[{\"name\":\"a\",\"proto\":\"ip\",\"port\":1}]}");
    cJSON *bad_range = cJSON_Parse(
        "{\"servers\":[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":99999}]}");
    cJSON *too_many = cJSON_Parse(
        "{\"servers\":[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":1},"
                      "{\"name\":\"b\",\"proto\":\"tcp\",\"port\":2},"
                      "{\"name\":\"c\",\"proto\":\"tcp\",\"port\":3},"
                      "{\"name\":\"d\",\"proto\":\"tcp\",\"port\":4},"
                      "{\"name\":\"e\",\"proto\":\"tcp\",\"port\":5}]}");

    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_validate(schema, good, NULL,
                                                 err, sizeof(err)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_validate(schema, missing_req, NULL,
                                         err, sizeof(err)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_validate(schema, bad_enum, NULL,
                                         err, sizeof(err)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_validate(schema, bad_range, NULL,
                                         err, sizeof(err)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_validate(schema, too_many, NULL,
                                         err, sizeof(err)));

    cJSON_Delete(good);
    cJSON_Delete(missing_req);
    cJSON_Delete(bad_enum);
    cJSON_Delete(bad_range);
    cJSON_Delete(too_many);
    free(schema);
}

void test_fields_array_defaults_collected(void)
{
    char  *schema = build_array();
    cJSON *defs = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_collect_defaults(schema, &defs));

    cJSON *servers = cJSON_GetObjectItemCaseSensitive(defs, "servers");

    TEST_ASSERT_TRUE(cJSON_IsArray(servers));
    TEST_ASSERT_EQUAL_INT(1, cJSON_GetArraySize(servers));
    TEST_ASSERT_EQUAL_STRING("obd0", cJSON_GetObjectItemCaseSensitive(
        cJSON_GetArrayItem(servers, 0), "name")->valuestring);
    TEST_ASSERT_TRUE(cJSON_IsArray(
        cJSON_GetObjectItemCaseSensitive(defs, "rules")));

    cJSON_Delete(defs);
    free(schema);
}

void test_fields_settings_json_stringize(void)
{
    /* plain JSON in, valid JSON text out; a nested JSON-in-JSON string
       keeps its (single) JSON escaping level through stringization */
    static const char *J = SETTINGS_JSON([
        {"payload":"{\"k\":\"v_${x}\"}","n":1}
    ]);
    cJSON *arr = cJSON_Parse(J);

    TEST_ASSERT_TRUE(cJSON_IsArray(arr));
    TEST_ASSERT_EQUAL_STRING("{\"k\":\"v_${x}\"}",
        cJSON_GetObjectItemCaseSensitive(cJSON_GetArrayItem(arr, 0),
                                         "payload")->valuestring);
    TEST_ASSERT_EQUAL_INT(1, (int)cJSON_GetObjectItemCaseSensitive(
        cJSON_GetArrayItem(arr, 0), "n")->valuedouble);
    cJSON_Delete(arr);
}

void test_fields_array_rejects_malformed_tables(void)
{
    char *out = NULL;

    /* maxItems missing / over the validator's ceiling */
    settings_field_t unbounded[] = { SETTINGS_ARRAY_ANY("a", 0, NULL) };
    settings_field_t oversize[] = { SETTINGS_ARRAY_ANY("a", 17, NULL) };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(unbounded, 1, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(oversize, 1, &out));

    /* nested arrays: one level only, matching the validator */
    static const settings_field_t inner[] = { SETTINGS_ARRAY_ANY("in", 4, NULL) };
    settings_field_t nested[] = { SETTINGS_ARRAY("a", 4, inner, NULL) };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(nested, 1, &out));

    /* default must parse as an array and fit maxItems */
    settings_field_t bad_json[] = { SETTINGS_ARRAY_ANY("a", 4, "{nope") };
    settings_field_t not_array[] = { SETTINGS_ARRAY_ANY("a", 4, "{}") };
    settings_field_t too_many[] =
        { SETTINGS_ARRAY_ANY("a", 1, "[{},{}]") };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(bad_json, 1, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(not_array, 1, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(too_many, 1, &out));

    /* array-only members on a scalar row */
    settings_field_t stray[] =
    {
        { "s", SETTINGS_FIELD_STRING, 0, 8, NULL, "", 0, false, false, NULL,
          NULL, 0, "[]" },
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_from_fields(stray, 1, &out));
}

void test_fields_required_emitted(void)
{
    settings_field_t f[] =
    {
        { "must", SETTINGS_FIELD_STRING, 0, 8, NULL, "", 0, false, true, NULL,
          NULL, 0, NULL },
        SETTINGS_BOOL("opt", false),
    };
    char *schema = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, sm_schema_from_fields(f, 2, &schema));

    cJSON *root = cJSON_Parse(schema);
    cJSON *req = cJSON_GetObjectItemCaseSensitive(root, "required");

    TEST_ASSERT_EQUAL_INT(1, cJSON_GetArraySize(req));
    TEST_ASSERT_EQUAL_STRING("must",
                             cJSON_GetArrayItem(req, 0)->valuestring);

    /* validator round trip: omitting a required key must fail */
    cJSON *missing = cJSON_Parse("{\"opt\":true}");
    char   err[96];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_schema_validate(schema, missing, NULL,
                                         err, sizeof(err)));

    cJSON_Delete(missing);
    cJSON_Delete(root);
    free(schema);
}

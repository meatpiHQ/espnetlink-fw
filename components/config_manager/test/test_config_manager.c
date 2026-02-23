#include "config_manager.h"
#include "filesystem.h"

#include "unity.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define CONFIG_PATH      FS_BASE_PATH "/config.json"
#define CONFIG_TMP_PATH  FS_BASE_PATH "/config.json.tmp"

static bool s_is_ready = false;

static void ensure_ready(void)
{
    if (s_is_ready)
    {
        return;
    }

    TEST_ASSERT_EQUAL(ESP_OK, filesystem_init());

    (void)remove(CONFIG_PATH);
    (void)remove(CONFIG_TMP_PATH);

    TEST_ASSERT_EQUAL(ESP_OK, config_manager_init());
    s_is_ready = true;
}

TEST_CASE("config_manager end-to-end sanity", "[config_manager]")
{
    ensure_ready();

    char value_buf[96] = {0};
    bool bool_value = false;

    TEST_ASSERT_EQUAL(ESP_OK, config_get_str("APN", value_buf, sizeof(value_buf)));
    TEST_ASSERT_EQUAL_STRING("internet", value_buf);

    TEST_ASSERT_EQUAL(ESP_OK, config_get_bool("LTE_ENABLED", &bool_value));
    TEST_ASSERT_TRUE(bool_value);

    TEST_ASSERT_EQUAL(ESP_OK, config_set_str("APN", "my_apn"));
    TEST_ASSERT_EQUAL(ESP_OK, config_set_from_str("LTE_ENABLED", "false"));
    TEST_ASSERT_EQUAL(ESP_OK, config_save());

    memset(value_buf, 0, sizeof(value_buf));
    TEST_ASSERT_EQUAL(ESP_OK, config_get_str("APN", value_buf, sizeof(value_buf)));
    TEST_ASSERT_EQUAL_STRING("my_apn", value_buf);

    TEST_ASSERT_EQUAL(ESP_OK, config_get_bool("LTE_ENABLED", &bool_value));
    TEST_ASSERT_FALSE(bool_value);

    TEST_ASSERT_EQUAL(ESP_OK, config_set_str("APN", "my_apn"));
    TEST_ASSERT_EQUAL(ESP_OK, config_set_from_str("LTE_ENABLED", "false"));
    TEST_ASSERT_EQUAL(ESP_OK, config_save());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, config_set_from_str("LTE_ENABLED", "not_a_bool"));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, config_get_as_str("NO_SUCH_KEY", value_buf, sizeof(value_buf)));

    TEST_ASSERT_EQUAL(ESP_OK, config_reset_defaults());

    memset(value_buf, 0, sizeof(value_buf));
    TEST_ASSERT_EQUAL(ESP_OK, config_get_str("APN", value_buf, sizeof(value_buf)));
    TEST_ASSERT_EQUAL_STRING("internet", value_buf);

    TEST_ASSERT_EQUAL(ESP_OK, config_get_bool("LTE_ENABLED", &bool_value));
    TEST_ASSERT_TRUE(bool_value);
}

TEST_CASE("config_manager persists wrapper fields", "[config_manager]")
{
    ensure_ready();

    TEST_ASSERT_EQUAL(ESP_OK, config_set_str("APN", "wrapper_test"));
    TEST_ASSERT_EQUAL(ESP_OK, config_save());

    FILE *file = fopen(CONFIG_PATH, "r");
    TEST_ASSERT_NOT_NULL(file);

    char file_buf[512] = {0};
    size_t bytes_read = fread(file_buf, 1, sizeof(file_buf) - 1, file);
    TEST_ASSERT_TRUE(bytes_read > 0);

    TEST_ASSERT_EQUAL_INT(0, fclose(file));

    TEST_ASSERT_NOT_NULL(strstr(file_buf, "\"version\""));
    TEST_ASSERT_NOT_NULL(strstr(file_buf, "\"config\""));
    TEST_ASSERT_NOT_NULL(strstr(file_buf, "\"crc32\""));
}

TEST_CASE("config_manager recovers from corrupted crc", "[config_manager]")
{
    ensure_ready();

    TEST_ASSERT_EQUAL(ESP_OK, config_set_str("APN", "before_corrupt"));
    TEST_ASSERT_EQUAL(ESP_OK, config_save());

    const char *corrupt_json =
        "{"
        "\"version\":1,"
        "\"config\":{"
        "\"APN\":\"bad_from_file\","
        "\"APN_USER\":\"\","
        "\"APN_PASS\":\"\","
        "\"DEVICE_NAME\":\"ESPNetLink\","
        "\"LTE_ENABLED\":true,"
        "\"GPS_ENABLED\":true"
        "},"
        "\"crc32\":0"
        "}";

    FILE *file = fopen(CONFIG_PATH, "w");
    TEST_ASSERT_NOT_NULL(file);
    TEST_ASSERT_TRUE(fputs(corrupt_json, file) >= 0);
    TEST_ASSERT_EQUAL_INT(0, fclose(file));

    TEST_ASSERT_EQUAL(ESP_OK, config_manager_init());

    char value_buf[96] = {0};
    TEST_ASSERT_EQUAL(ESP_OK, config_get_str("APN", value_buf, sizeof(value_buf)));
    TEST_ASSERT_EQUAL_STRING("internet", value_buf);
}

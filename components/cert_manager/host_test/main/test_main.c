/**
 * @file test_main.c
 * @brief Host tests for cert_manager's pure policy: set-name validation
 *        (traversal safety), part/legacy-field mapping, PEM plausibility.
 *        Expected: 6 Tests 0 Failures 0 Ignored.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "cert_manager_private.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_set_names(void)
{
    TEST_ASSERT_TRUE(cm_set_name_valid("home-broker"));
    TEST_ASSERT_TRUE(cm_set_name_valid("aws_iot_1"));
    TEST_ASSERT_FALSE(cm_set_name_valid(""));
    TEST_ASSERT_FALSE(cm_set_name_valid(NULL));
    TEST_ASSERT_FALSE(cm_set_name_valid("../../../data/settings"));
    TEST_ASSERT_FALSE(cm_set_name_valid("has space"));
    TEST_ASSERT_FALSE(cm_set_name_valid("UPPER"));
    TEST_ASSERT_FALSE(cm_set_name_valid("a-name-way-too-long-for-a-set"));
}

static void test_part_mapping(void)
{
    cert_manager_part_t part;

    TEST_ASSERT_EQUAL_STRING("ca.pem", cm_part_filename(CERT_MANAGER_CA));
    TEST_ASSERT_EQUAL_STRING("client.crt",
                             cm_part_filename(CERT_MANAGER_CLIENT_CERT));
    TEST_ASSERT_EQUAL_STRING("client.key",
                             cm_part_filename(CERT_MANAGER_CLIENT_KEY));
    TEST_ASSERT_TRUE(cm_part_from_type("ca", &part));
    TEST_ASSERT_EQUAL_INT(CERT_MANAGER_CA, part);
    TEST_ASSERT_TRUE(cm_part_from_type("key", &part));
    TEST_ASSERT_EQUAL_INT(CERT_MANAGER_CLIENT_KEY, part);
    TEST_ASSERT_FALSE(cm_part_from_type("pem", &part));
    TEST_ASSERT_FALSE(cm_part_from_type(NULL, &part));
}

static void test_legacy_field_mapping(void)
{
    /* the legacy multipart form's field names must keep working */
    cert_manager_part_t part;

    TEST_ASSERT_TRUE(cm_part_from_field("ca", &part));
    TEST_ASSERT_EQUAL_INT(CERT_MANAGER_CA, part);
    TEST_ASSERT_TRUE(cm_part_from_field("client_cert", &part));
    TEST_ASSERT_EQUAL_INT(CERT_MANAGER_CLIENT_CERT, part);
    TEST_ASSERT_TRUE(cm_part_from_field("client_key", &part));
    TEST_ASSERT_EQUAL_INT(CERT_MANAGER_CLIENT_KEY, part);
    TEST_ASSERT_FALSE(cm_part_from_field("cert", &part)); /* type != field */
    TEST_ASSERT_FALSE(cm_part_from_field("firmware", &part));
}

static const char CERT_PEM[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIBszCCAVmgAwIBAgIUfake0000000000000000000000000000000000000\n"
    "-----END CERTIFICATE-----\n";
static const char KEY_PEM[] =
    "-----BEGIN EC PRIVATE KEY-----\n"
    "MHcCAQEEIFake000000000000000000000000000000000000000000000000\n"
    "-----END EC PRIVATE KEY-----\n";

static void test_pem_accepts_right_kinds(void)
{
    TEST_ASSERT_TRUE(cm_pem_plausible(CERT_MANAGER_CA, CERT_PEM,
                                      sizeof(CERT_PEM) - 1));
    TEST_ASSERT_TRUE(cm_pem_plausible(CERT_MANAGER_CLIENT_CERT, CERT_PEM,
                                      sizeof(CERT_PEM) - 1));
    TEST_ASSERT_TRUE(cm_pem_plausible(CERT_MANAGER_CLIENT_KEY, KEY_PEM,
                                      sizeof(KEY_PEM) - 1));
}

static void test_pem_rejects_wrong_kind(void)
{
    /* a certificate uploaded as the KEY (and vice versa) is the classic
     * wrong-file mistake */
    TEST_ASSERT_FALSE(cm_pem_plausible(CERT_MANAGER_CLIENT_KEY, CERT_PEM,
                                       sizeof(CERT_PEM) - 1));
    TEST_ASSERT_FALSE(cm_pem_plausible(CERT_MANAGER_CA, KEY_PEM,
                                       sizeof(KEY_PEM) - 1));
}

static void test_pem_rejects_binary_and_bounds(void)
{
    char der[64];

    memset(der, 0x30, sizeof(der));
    der[1] = (char)0x82; /* DER SEQUENCE — binary upload */
    TEST_ASSERT_FALSE(cm_pem_plausible(CERT_MANAGER_CA, der, sizeof(der)));
    TEST_ASSERT_FALSE(cm_pem_plausible(CERT_MANAGER_CA, CERT_PEM, 8));
    TEST_ASSERT_FALSE(cm_pem_plausible(CERT_MANAGER_CA, NULL, 100));
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_set_names);
    RUN_TEST(test_part_mapping);
    RUN_TEST(test_legacy_field_mapping);
    RUN_TEST(test_pem_accepts_right_kinds);
    RUN_TEST(test_pem_rejects_wrong_kind);
    RUN_TEST(test_pem_rejects_binary_and_bounds);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}

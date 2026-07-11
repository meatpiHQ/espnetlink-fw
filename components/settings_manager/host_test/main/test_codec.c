/**
 * @file test_codec.c
 * @brief Host (linux target) unit tests for the envelope codec and CRC-32.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"
#include "cJSON.h"
#include "settings_manager_private.h"

void test_crc32_known_vector(void)
{
    /* CRC-32/ISO-HDLC of "123456789" is 0xCBF43926. */
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u,
                            sm_crc32((const uint8_t *)"123456789", 9));
}

void test_encode_decode_roundtrip(void)
{
    cJSON *in = cJSON_Parse("{\"a\":1,\"b\":\"x\",\"c\":true}");
    char  *env = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, sm_codec_encode(7, in, &env));
    TEST_ASSERT_NOT_NULL(env);

    uint32_t ver = 0;
    cJSON   *out = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, sm_codec_decode(env, &ver, &out));
    TEST_ASSERT_EQUAL_UINT32(7, ver);
    TEST_ASSERT_TRUE(cJSON_Compare(in, out, true));

    free(env);
    cJSON_Delete(in);
    cJSON_Delete(out);
}

void test_decode_detects_tamper(void)
{
    cJSON *in = cJSON_Parse("{\"a\":1}");
    char  *env = NULL;
    sm_codec_encode(1, in, &env);

    /* Flip a byte inside the data so the stored CRC no longer matches. */
    char *one = strstr(env, "\"a\":1");
    TEST_ASSERT_NOT_NULL(one);
    one[4] = '2';

    cJSON *out = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, sm_codec_decode(env, NULL, &out));

    free(env);
    cJSON_Delete(in);
}

void test_decode_rejects_malformed(void)
{
    cJSON *out = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sm_codec_decode("not json", NULL, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      sm_codec_decode("{\"data\":{}}", NULL, &out));   /* no crc32 */
}

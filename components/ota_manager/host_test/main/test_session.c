/**
 * @file test_session.c
 * @brief Host tests for the pure OTA session state machine: every
 *        transition rule, error latching, and backend-call accounting —
 *        against a recorder backend (no esp_ota).
 */
#include <string.h>

#include "unity.h"

#include "ota_manager_private.h"

/* ---- recorder backend ----------------------------------------------------------- */

typedef struct
{
    int begins;
    int writes;
    int ends;
    int aborts;
    size_t bytes;
    esp_err_t begin_rc;
    esp_err_t write_rc;
    esp_err_t end_rc;
} rec_t;

static rec_t s_rec;

static esp_err_t rec_begin(void *io, size_t total)
{
    (void)io;
    (void)total;
    s_rec.begins++;
    return s_rec.begin_rc;
}

static esp_err_t rec_write(void *io, const void *d, size_t l)
{
    (void)io;
    (void)d;
    s_rec.writes++;
    s_rec.bytes += l;
    return s_rec.write_rc;
}

static esp_err_t rec_end(void *io)
{
    (void)io;
    s_rec.ends++;
    return s_rec.end_rc;
}

static void rec_abort(void *io)
{
    (void)io;
    s_rec.aborts++;
}

static const ota_session_ops_t REC_OPS =
{
    .begin = rec_begin,
    .write = rec_write,
    .end = rec_end,
    .abort = rec_abort,
};

static ota_session_t s_s;

static void fresh(void)
{
    memset(&s_rec, 0, sizeof(s_rec));
    ota_session_init(&s_s, &REC_OPS, NULL);
}

/* ---- tests ------------------------------------------------------------------------ */

static void test_happy_path_known_size(void)
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_OK, ota_session_begin(&s_s, 8));
    TEST_ASSERT_EQUAL(OTA_MANAGER_RECEIVING, s_s.state);
    TEST_ASSERT_EQUAL(ESP_OK, ota_session_write(&s_s, "abcd", 4));
    TEST_ASSERT_EQUAL(ESP_OK, ota_session_write(&s_s, "efgh", 4));
    TEST_ASSERT_EQUAL(ESP_OK, ota_session_end(&s_s));
    TEST_ASSERT_EQUAL(OTA_MANAGER_READY, s_s.state);
    TEST_ASSERT_EQUAL(8, s_s.received);
    TEST_ASSERT_EQUAL(1, s_rec.ends);
    TEST_ASSERT_EQUAL(0, s_rec.aborts);
}

static void test_write_before_begin_rejected(void)
{
    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      ota_session_write(&s_s, "x", 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_session_end(&s_s));
    TEST_ASSERT_EQUAL(0, s_rec.writes);
}

static void test_double_begin_rejected_while_receiving(void)
{
    fresh();
    ota_session_begin(&s_s, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ota_session_begin(&s_s, 0));
    TEST_ASSERT_EQUAL(1, s_rec.begins); /* second never hit the backend */
}

static void test_overrun_of_announced_size_fails(void)
{
    fresh();
    ota_session_begin(&s_s, 4);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE,
                      ota_session_write(&s_s, "toolong", 7));
    TEST_ASSERT_EQUAL(OTA_MANAGER_FAILED, s_s.state);
    TEST_ASSERT_EQUAL(1, s_rec.aborts); /* flash session released */
    TEST_ASSERT_NOT_NULL(strstr(s_s.error, "larger"));
}

static void test_short_image_fails_at_end(void)
{
    fresh();
    ota_session_begin(&s_s, 100);
    ota_session_write(&s_s, "half", 4);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, ota_session_end(&s_s));
    TEST_ASSERT_EQUAL(OTA_MANAGER_FAILED, s_s.state);
    TEST_ASSERT_NOT_NULL(strstr(s_s.error, "short"));
    TEST_ASSERT_EQUAL(0, s_rec.ends); /* backend end never attempted */
}

static void test_empty_image_fails_at_end(void)
{
    fresh();
    ota_session_begin(&s_s, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, ota_session_end(&s_s));
    TEST_ASSERT_NOT_NULL(strstr(s_s.error, "empty"));
}

static void test_backend_write_failure_latches_and_aborts(void)
{
    fresh();
    ota_session_begin(&s_s, 0);
    s_rec.write_rc = ESP_FAIL;
    TEST_ASSERT_EQUAL(ESP_FAIL, ota_session_write(&s_s, "x", 1));
    TEST_ASSERT_EQUAL(OTA_MANAGER_FAILED, s_s.state);
    TEST_ASSERT_EQUAL(1, s_rec.aborts);
}

static void test_validation_failure_no_double_abort(void)
{
    fresh();
    ota_session_begin(&s_s, 0);
    ota_session_write(&s_s, "data", 4);
    s_rec.end_rc = ESP_FAIL; /* esp_ota_end releases the handle itself */
    TEST_ASSERT_EQUAL(ESP_FAIL, ota_session_end(&s_s));
    TEST_ASSERT_EQUAL(OTA_MANAGER_FAILED, s_s.state);
    TEST_ASSERT_EQUAL(0, s_rec.aborts);
    TEST_ASSERT_NOT_NULL(strstr(s_s.error, "validation"));
}

static void test_retry_after_failure_and_abort_resets(void)
{
    fresh();
    ota_session_begin(&s_s, 0);
    s_rec.write_rc = ESP_FAIL;
    ota_session_write(&s_s, "x", 1); /* -> FAILED */
    s_rec.write_rc = ESP_OK;

    /* begin from FAILED = retry */
    TEST_ASSERT_EQUAL(ESP_OK, ota_session_begin(&s_s, 0));
    TEST_ASSERT_EQUAL(ESP_OK, ota_session_write(&s_s, "ok", 2));

    /* abort from RECEIVING -> IDLE + backend abort */
    TEST_ASSERT_EQUAL(ESP_OK, ota_session_abort(&s_s));
    TEST_ASSERT_EQUAL(OTA_MANAGER_IDLE, s_s.state);
    TEST_ASSERT_EQUAL(0, s_s.received);
    TEST_ASSERT_EQUAL(2, s_rec.aborts);
}

void run_session_tests(void)
{
    RUN_TEST(test_happy_path_known_size);
    RUN_TEST(test_write_before_begin_rejected);
    RUN_TEST(test_double_begin_rejected_while_receiving);
    RUN_TEST(test_overrun_of_announced_size_fails);
    RUN_TEST(test_short_image_fails_at_end);
    RUN_TEST(test_empty_image_fails_at_end);
    RUN_TEST(test_backend_write_failure_latches_and_aborts);
    RUN_TEST(test_validation_failure_no_double_abort);
    RUN_TEST(test_retry_after_failure_and_abort_resets);
}

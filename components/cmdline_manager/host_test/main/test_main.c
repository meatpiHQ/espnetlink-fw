/**
 * @file test_main.c
 * @brief Host tests for cmdline_manager's pure line assembler — every
 *        transport chunking pattern (whole lines, split lines, CRLF,
 *        batches, oversize floods) must come out as clean lines.
 *        Expected: 6 Tests 0 Failures 0 Ignored.
 */
#include <string.h>

#include "unity.h"

#include "cmdline_manager_private.h"

static cm_line_asm_t s_asm;

void setUp(void)
{
    cm_line_reset(&s_asm);
}

void tearDown(void)
{
}

/** Feed everything, collecting completed lines into @p out[]. */
static int feed_all(const char *bytes, const char *out[], int max)
{
    size_t len = strlen(bytes);
    size_t off = 0;
    int found = 0;
    static char store[8][256];

    while (off < len)
    {
        const char *line = NULL;

        off += cm_line_feed(&s_asm, (const uint8_t *)bytes + off,
                            len - off, &line);

        if (line != NULL && found < max)
        {
            snprintf(store[found], sizeof(store[found]), "%s", line);
            out[found] = store[found];
            found++;
        }
    }

    return found;
}

static void test_simple_line(void)
{
    const char *lines[4];

    TEST_ASSERT_EQUAL_INT(1, feed_all("version\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("version", lines[0]);
}

static void test_crlf_yields_one_line(void)
{
    const char *lines[4];

    /* the \r terminates; the \n tail must NOT become an empty line */
    TEST_ASSERT_EQUAL_INT(1, feed_all("status\r\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("status", lines[0]);
}

static void test_split_across_chunks(void)
{
    const char *line = NULL;

    /* transports chunk arbitrarily — "ver" now, "sion\n" later */
    cm_line_feed(&s_asm, (const uint8_t *)"ver", 3, &line);
    TEST_ASSERT_NULL(line);
    cm_line_feed(&s_asm, (const uint8_t *)"sion\n", 5, &line);
    TEST_ASSERT_NOT_NULL(line);
    TEST_ASSERT_EQUAL_STRING("version", line);
}

static void test_batch_of_lines_one_chunk(void)
{
    const char *lines[4];

    TEST_ASSERT_EQUAL_INT(3, feed_all("help\nrtc sync\r\nimu\n",
                                      lines, 4));
    TEST_ASSERT_EQUAL_STRING("help", lines[0]);
    TEST_ASSERT_EQUAL_STRING("rtc sync", lines[1]);
    TEST_ASSERT_EQUAL_STRING("imu", lines[2]);
}

static void test_empty_lines_skipped(void)
{
    const char *lines[4];

    /* a terminal sending bare newlines must not run empty commands */
    TEST_ASSERT_EQUAL_INT(1, feed_all("\n\r\n\nled\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("led", lines[0]);
}

static void test_overflow_discarded_then_recovers(void)
{
    static char flood[400];
    const char *lines[4];

    memset(flood, 'x', sizeof(flood) - 2);
    flood[sizeof(flood) - 2] = '\n';
    flood[sizeof(flood) - 1] = '\0';

    /* an oversize line (buf is 256) is dropped whole... */
    TEST_ASSERT_EQUAL_INT(0, feed_all(flood, lines, 4));
    /* ...and the assembler is clean for the next command */
    TEST_ASSERT_EQUAL_INT(1, feed_all("battery\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("battery", lines[0]);
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_simple_line);
    RUN_TEST(test_crlf_yields_one_line);
    RUN_TEST(test_split_across_chunks);
    RUN_TEST(test_batch_of_lines_one_chunk);
    RUN_TEST(test_empty_lines_skipped);
    RUN_TEST(test_overflow_discarded_then_recovers);
    UNITY_END();
}

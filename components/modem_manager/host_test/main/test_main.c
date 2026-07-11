/**
 * @file test_main.c
 * @brief Host tests for modem_manager's pure AT-response parsers
 *        (modem_manager_parse.c): AT+IFC? / AT+IPR? payloads as the probe
 *        layer hands them over (prefix already stripped).
 */
#include <stdlib.h>

#include "unity.h"

#include "modem_manager_parse.h"

void test_parse_ifc_accepts_pair(void)
{
    int rx = -1, tx = -1;

    TEST_ASSERT_TRUE(mm_parse_ifc("2,2", &rx, &tx));
    TEST_ASSERT_EQUAL(2, rx);
    TEST_ASSERT_EQUAL(2, tx);

    TEST_ASSERT_TRUE(mm_parse_ifc("0,1", &rx, &tx));
    TEST_ASSERT_EQUAL(0, rx);
    TEST_ASSERT_EQUAL(1, tx);

    /* %d skips leading whitespace (before each number only) */
    TEST_ASSERT_TRUE(mm_parse_ifc(" 2, 0", &rx, &tx));
    TEST_ASSERT_EQUAL(2, rx);
    TEST_ASSERT_EQUAL(0, tx);
}

void test_parse_ifc_rejects_incomplete(void)
{
    int rx = -1, tx = -1;

    TEST_ASSERT_FALSE(mm_parse_ifc("2", &rx, &tx));      /* one value   */
    TEST_ASSERT_FALSE(mm_parse_ifc("", &rx, &tx));       /* empty       */
    TEST_ASSERT_FALSE(mm_parse_ifc("OK", &rx, &tx));     /* status line */
    /* an unstripped "+IFC: 2,2" must NOT parse — the probe layer strips
     * the prefix before calling */
    TEST_ASSERT_FALSE(mm_parse_ifc("+IFC: 2,2", &rx, &tx));
    /* whitespace BEFORE the comma breaks sscanf's literal ',' match; the
     * probe layer never produces it — documented contract, not a bug */
    TEST_ASSERT_FALSE(mm_parse_ifc("2 ,2", &rx, &tx));
}

void test_parse_ipr_accepts_bauds(void)
{
    int baud = -1;

    TEST_ASSERT_TRUE(mm_parse_ipr("115200", &baud));
    TEST_ASSERT_EQUAL(115200, baud);

    TEST_ASSERT_TRUE(mm_parse_ipr("3000000", &baud));
    TEST_ASSERT_EQUAL(3000000, baud);

    /* trailing CR from the AT line reader is harmless */
    TEST_ASSERT_TRUE(mm_parse_ipr("  921600\r", &baud));
    TEST_ASSERT_EQUAL(921600, baud);
}

void test_parse_ipr_rejects_non_numeric(void)
{
    int baud = -1;

    TEST_ASSERT_FALSE(mm_parse_ipr("", &baud));
    TEST_ASSERT_FALSE(mm_parse_ipr("ERROR", &baud));
    TEST_ASSERT_FALSE(mm_parse_ipr("+IPR: 115200", &baud));
}

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_parse_ifc_accepts_pair);
    RUN_TEST(test_parse_ifc_rejects_incomplete);
    RUN_TEST(test_parse_ipr_accepts_bauds);
    RUN_TEST(test_parse_ipr_rejects_non_numeric);

    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}

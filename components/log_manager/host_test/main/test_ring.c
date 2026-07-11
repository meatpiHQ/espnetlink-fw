/**
 * @file test_ring.c
 * @brief Unit tests for the pure PSRAM crash-ring operations.
 */
#include <string.h>

#include "unity.h"

#include "log_manager_private.h"

#define RING_SIZE 32u

static lm_ring_hdr_t s_hdr;
static uint8_t s_buf[RING_SIZE];
static char s_out[RING_SIZE + 1];

static void append(const char *s)
{
    lm_ring_append(&s_hdr, s_buf, s, strlen(s));
}

static size_t read_all(void)
{
    size_t n = lm_ring_read(&s_hdr, s_buf, s_out, sizeof(s_out) - 1);

    s_out[n] = '\0';
    return n;
}

void test_garbage_header_is_invalid(void)
{
    memset(&s_hdr, 0xC3, sizeof(s_hdr)); /* power-on garbage */
    TEST_ASSERT_FALSE(lm_ring_valid(&s_hdr, RING_SIZE));
}

void test_reset_then_valid(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);
    TEST_ASSERT_TRUE(lm_ring_valid(&s_hdr, RING_SIZE));
    TEST_ASSERT_EQUAL_UINT32(0, s_hdr.used);
    TEST_ASSERT_EQUAL(0, read_all());
}

void test_append_and_read_roundtrip(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);
    append("hello ");
    append("world");
    TEST_ASSERT_EQUAL(11, read_all());
    TEST_ASSERT_EQUAL_STRING("hello world", s_out);
    TEST_ASSERT_TRUE(lm_ring_valid(&s_hdr, RING_SIZE));
}

void test_wrap_keeps_newest(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);

    /* 4 x 10 bytes = 40 > 32: the oldest 8 bytes fall off */
    append("AAAAAAAAA\n");
    append("BBBBBBBBB\n");
    append("CCCCCCCCC\n");
    append("DDDDDDDDD\n");

    TEST_ASSERT_EQUAL(RING_SIZE, read_all());
    /* 40 bytes into a 32-byte ring keeps bytes 8..39: the D line is intact
       and at most ONE trailing 'A' (byte 8) survives — never the full run */
    TEST_ASSERT_NOT_NULL(strstr(s_out, "DDDDDDDDD"));
    TEST_ASSERT_NULL(strstr(s_out, "AA"));
    TEST_ASSERT_EQUAL('D', s_out[RING_SIZE - 2]); /* newest byte before \n */
}

void test_oversized_append_keeps_tail(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);

    char big[100];

    memset(big, 'x', sizeof(big));
    memcpy(big + sizeof(big) - 4, "END", 4); /* incl. NUL */
    lm_ring_append(&s_hdr, s_buf, big, sizeof(big));

    TEST_ASSERT_EQUAL(RING_SIZE, read_all());
    TEST_ASSERT_EQUAL_STRING("END", s_out + RING_SIZE - 4);
}

void test_read_into_small_buffer_gets_newest(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);
    append("0123456789");

    char small[5];
    size_t n = lm_ring_read(&s_hdr, s_buf, small, 4);

    small[n] = '\0';
    TEST_ASSERT_EQUAL(4, n);
    TEST_ASSERT_EQUAL_STRING("6789", small); /* newest, chronological */
}

void test_header_tamper_detected(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);
    append("abc");
    TEST_ASSERT_TRUE(lm_ring_valid(&s_hdr, RING_SIZE));

    s_hdr.head ^= 1u;
    TEST_ASSERT_FALSE(lm_ring_valid(&s_hdr, RING_SIZE));
}

void test_size_mismatch_invalid(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);
    TEST_ASSERT_FALSE(lm_ring_valid(&s_hdr, RING_SIZE * 2));
}

void test_tuning_guard_clobber_is_harmless(void)
{
    lm_ring_reset(&s_hdr, RING_SIZE);
    append("keep me");

    /* S3 MSPI timing tuning writes the first 64 bytes each boot — the guard */
    memset(s_hdr.mspi_tuning_guard, 0x5A, sizeof(s_hdr.mspi_tuning_guard));
    TEST_ASSERT_TRUE(lm_ring_valid(&s_hdr, RING_SIZE));
    TEST_ASSERT_EQUAL(7, read_all());
    TEST_ASSERT_EQUAL_STRING("keep me", s_out);
}

/**
 * @file test_core.c
 * @brief Unit tests for the pure restart-tracking state machine.
 */
#include <string.h>

#include "unity.h"

#include "restart_tracker_private.h"

#define RST_POWERON   1U
#define RST_EXT       2U
#define RST_SW        3U
#define RST_PANIC     4U
#define RST_INTWDT    5U
#define RST_TASKWDT   6U  /* esp_reset_reason_t: 8 is DEEPSLEEP, not a wdt */
#define RST_DEEPSLEEP 8U
#define RST_BROWNOUT  9U
#define RST_USB       11U
#define RST_JTAG      12U

#define GOOD_TIME 1780000000LL /* well past the 2024 sanity floor */

static restart_tracker_state_t s_state;

static rt_inputs_t in(uint32_t reason, int64_t now)
{
    rt_inputs_t i = { .now_unix = now, .uptime_ms = 1234, .reset_reason = reason };

    return i;
}

void test_random_memory_is_invalid_and_resets(void)
{
    memset(&s_state, 0xA5, sizeof(s_state)); /* power-on garbage */
    TEST_ASSERT_FALSE(rt_state_is_valid(&s_state));

    rt_inputs_t i = in(RST_POWERON, GOOD_TIME);

    TEST_ASSERT_TRUE(rt_record_boot(&s_state, &i)); /* reports fresh reset */
    TEST_ASSERT_TRUE(rt_state_is_valid(&s_state));
    TEST_ASSERT_EQUAL_UINT32(1, s_state.boot_count);
}

void test_boot_recording_and_counters(void)
{
    memset(&s_state, 0, sizeof(s_state));

    rt_inputs_t i = in(RST_SW, GOOD_TIME);

    rt_record_boot(&s_state, &i); /* first boot: adopts fresh state */
    TEST_ASSERT_EQUAL_UINT32(1, s_state.boot_count);

    /* second boot keeps the valid state (no reset reported) */
    TEST_ASSERT_FALSE(rt_record_boot(&s_state, &i));
    TEST_ASSERT_EQUAL_UINT32(2, s_state.boot_count);

    const restart_tracker_record_t *rec =
        &s_state.history[s_state.latest_history_index];

    TEST_ASSERT_EQUAL_UINT32(2, rec->sequence);
    TEST_ASSERT_EQUAL(GOOD_TIME, rec->boot_timestamp);
    TEST_ASSERT_EQUAL_UINT8(1, rec->time_valid);
    TEST_ASSERT_EQUAL_UINT32(0, s_state.unexpected_reset_count); /* sw is planned-ish */
}

void test_planned_restart_consumed_once(void)
{
    memset(&s_state, 0, sizeof(s_state));
    rt_inputs_t boot = in(RST_SW, GOOD_TIME);

    rt_record_boot(&s_state, &boot);

    rt_inputs_t req = in(RST_SW, GOOD_TIME + 5);

    rt_mark_planned(&s_state, &req,
                    RESTART_TRACKER_PLANNED_REASON_CONFIG_APPLY,
                    RESTART_TRACKER_SOURCE_CONFIG_SERVER, 0x42);
    TEST_ASSERT_TRUE(s_state.pending_restart.valid);

    /* "reboot" */
    rt_record_boot(&s_state, &boot);

    const restart_tracker_record_t *rec =
        &s_state.history[s_state.latest_history_index];

    TEST_ASSERT_EQUAL_UINT8(1, rec->was_planned);
    TEST_ASSERT_EQUAL_UINT16(RESTART_TRACKER_PLANNED_REASON_CONFIG_APPLY,
                             rec->planned_reason);
    TEST_ASSERT_EQUAL_UINT16(RESTART_TRACKER_SOURCE_CONFIG_SERVER, rec->source);
    TEST_ASSERT_EQUAL_UINT32(0x42, rec->flags);
    TEST_ASSERT_EQUAL(GOOD_TIME + 5, rec->request_timestamp);
    TEST_ASSERT_FALSE(s_state.pending_restart.valid); /* consumed */

    /* next boot without a new mark: not planned */
    rt_record_boot(&s_state, &boot);
    rec = &s_state.history[s_state.latest_history_index];
    TEST_ASSERT_EQUAL_UINT8(0, rec->was_planned);
}

void test_unexpected_reason_classification(void)
{
    memset(&s_state, 0, sizeof(s_state));

    rt_inputs_t panic = in(RST_PANIC, GOOD_TIME);
    rt_inputs_t wdt = in(RST_TASKWDT, GOOD_TIME);
    rt_inputs_t sw = in(RST_SW, GOOD_TIME);

    rt_record_boot(&s_state, &panic);
    rt_record_boot(&s_state, &wdt);
    rt_record_boot(&s_state, &sw);
    TEST_ASSERT_EQUAL_UINT32(2, s_state.unexpected_reset_count);

    /* a PLANNED panic-reason boot never counts (was_planned wins) */
    rt_mark_planned(&s_state, &sw, RESTART_TRACKER_PLANNED_REASON_OTA_APPLY,
                    RESTART_TRACKER_SOURCE_OTA, 0);
    rt_record_boot(&s_state, &panic);
    TEST_ASSERT_EQUAL_UINT32(2, s_state.unexpected_reset_count);

    /* deliberate external causes (2026-10-15): EN pin, a deep-sleep wake,
     * the USB-Serial-JTAG reset esptool issues after a native-USB flash,
     * a debugger. None of them is a fault. */
    rt_inputs_t ext = in(RST_EXT, GOOD_TIME);
    rt_inputs_t deepsleep = in(RST_DEEPSLEEP, GOOD_TIME);
    rt_inputs_t usb = in(RST_USB, GOOD_TIME);
    rt_inputs_t jtag = in(RST_JTAG, GOOD_TIME);

    rt_record_boot(&s_state, &ext);
    rt_record_boot(&s_state, &deepsleep);
    rt_record_boot(&s_state, &usb);
    rt_record_boot(&s_state, &jtag);
    TEST_ASSERT_EQUAL_UINT32(2, s_state.unexpected_reset_count);

    /* faults the old table hid: the interrupt watchdog (5, which the WiCAN
     * copy took for deepsleep) and a brownout */
    rt_inputs_t intwdt = in(RST_INTWDT, GOOD_TIME);
    rt_inputs_t brownout = in(RST_BROWNOUT, GOOD_TIME);

    rt_record_boot(&s_state, &intwdt);
    rt_record_boot(&s_state, &brownout);
    TEST_ASSERT_EQUAL_UINT32(4, s_state.unexpected_reset_count);

    /* the names follow esp_reset_reason_t */
    TEST_ASSERT_EQUAL_STRING("interrupt_wdt", restart_tracker_reset_reason_to_str(RST_INTWDT));
    TEST_ASSERT_EQUAL_STRING("task_wdt", restart_tracker_reset_reason_to_str(RST_TASKWDT));
    TEST_ASSERT_EQUAL_STRING("deepsleep", restart_tracker_reset_reason_to_str(RST_DEEPSLEEP));
    TEST_ASSERT_EQUAL_STRING("brownout", restart_tracker_reset_reason_to_str(RST_BROWNOUT));
    TEST_ASSERT_EQUAL_STRING("usb", restart_tracker_reset_reason_to_str(RST_USB));
    TEST_ASSERT_EQUAL_STRING("invalid", restart_tracker_reset_reason_to_str(99U));
}

void test_history_ring_wraps(void)
{
    memset(&s_state, 0, sizeof(s_state));

    rt_inputs_t i = in(RST_SW, GOOD_TIME);

    for (int b = 0; b < RESTART_TRACKER_HISTORY_LEN + 3; b++)
    {
        rt_record_boot(&s_state, &i);
    }

    TEST_ASSERT_EQUAL_UINT32(RESTART_TRACKER_HISTORY_LEN + 3, s_state.boot_count);
    TEST_ASSERT_EQUAL_UINT32((RESTART_TRACKER_HISTORY_LEN + 2) %
                             RESTART_TRACKER_HISTORY_LEN,
                             s_state.latest_history_index);

    /* the latest record carries the newest sequence */
    TEST_ASSERT_EQUAL_UINT32(RESTART_TRACKER_HISTORY_LEN + 3,
                             s_state.history[s_state.latest_history_index].sequence);
}

void test_crc_detects_tamper(void)
{
    memset(&s_state, 0, sizeof(s_state));
    rt_inputs_t i = in(RST_SW, GOOD_TIME);

    rt_record_boot(&s_state, &i);
    TEST_ASSERT_TRUE(rt_state_is_valid(&s_state));

    s_state.boot_count ^= 0x80;
    TEST_ASSERT_FALSE(rt_state_is_valid(&s_state));
}

void test_tuning_guard_clobber_is_harmless(void)
{
    memset(&s_state, 0, sizeof(s_state));
    rt_inputs_t i = in(RST_SW, GOOD_TIME);

    rt_record_boot(&s_state, &i);

    /* the S3 MSPI timing tuning overwrites the first 64 bytes every boot —
       exactly the guard; the state must stay valid */
    memset(s_state.mspi_tuning_guard, 0xA5,
           sizeof(s_state.mspi_tuning_guard));
    TEST_ASSERT_TRUE(rt_state_is_valid(&s_state));
    TEST_ASSERT_EQUAL_UINT32(1, s_state.boot_count);
}

void test_invalid_time_stored_as_zero(void)
{
    memset(&s_state, 0, sizeof(s_state));

    rt_inputs_t i = in(RST_SW, 12345); /* pre-2024: clock not set yet */

    rt_record_boot(&s_state, &i);

    const restart_tracker_record_t *rec =
        &s_state.history[s_state.latest_history_index];

    TEST_ASSERT_EQUAL(0, rec->boot_timestamp);
    TEST_ASSERT_EQUAL_UINT8(0, rec->time_valid);
}

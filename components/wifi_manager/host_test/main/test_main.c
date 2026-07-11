/**
 * @file test_main.c
 * @brief Host test entry point for the wifi_manager selection module.
 */
#include "unity.h"

void test_scan_prefers_primary(void);
void test_scan_falls_back_in_priority_order(void);
void test_scan_skips_banned_candidate(void);
void test_scan_all_banned_throttled_retry(void);
void test_scan_unbanned_alternative_beats_throttle(void);
void test_scan_nothing_present_returns_none(void);
void test_ban_after_threshold_and_expiry(void);
void test_success_clears_failures(void);
void test_already_banned_does_not_extend(void);
void test_sequential_rotates_and_wraps(void);
void test_sequential_skips_banned(void);
void test_sequential_all_banned_throttled(void);
void test_sequential_first_pick_is_primary(void);
void test_single_network_banned_trickle(void);
void test_roam_better_available(void);
void test_duplicate_ssid_ban_covers_both(void);
void test_parse_ap_ipv4(void);
void test_parse_ipv4_plain(void);
void test_netmask_valid(void);
void test_backoff_curve(void);

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_scan_prefers_primary);
    RUN_TEST(test_scan_falls_back_in_priority_order);
    RUN_TEST(test_scan_skips_banned_candidate);
    RUN_TEST(test_scan_all_banned_throttled_retry);
    RUN_TEST(test_scan_unbanned_alternative_beats_throttle);
    RUN_TEST(test_scan_nothing_present_returns_none);
    RUN_TEST(test_ban_after_threshold_and_expiry);
    RUN_TEST(test_success_clears_failures);
    RUN_TEST(test_already_banned_does_not_extend);
    RUN_TEST(test_sequential_rotates_and_wraps);
    RUN_TEST(test_sequential_skips_banned);
    RUN_TEST(test_sequential_all_banned_throttled);
    RUN_TEST(test_sequential_first_pick_is_primary);
    RUN_TEST(test_single_network_banned_trickle);
    RUN_TEST(test_roam_better_available);
    RUN_TEST(test_duplicate_ssid_ban_covers_both);
    RUN_TEST(test_parse_ap_ipv4);
    RUN_TEST(test_parse_ipv4_plain);
    RUN_TEST(test_netmask_valid);
    RUN_TEST(test_backoff_curve);
    UNITY_END();
}

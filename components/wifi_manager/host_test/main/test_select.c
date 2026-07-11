/**
 * @file test_select.c
 * @brief Unit tests for the pure STA candidate selection + ban list.
 */
#include <string.h>

#include "unity.h"

#include "wifi_manager_private.h"

static wm_select_state_t s_st;
static wm_network_t      s_cand[3];

static void reset(void)
{
    wm_select_init(&s_st);
    memset(s_cand, 0, sizeof(s_cand));
    strcpy(s_cand[0].ssid, "primary");
    strcpy(s_cand[1].ssid, "fb1");
    strcpy(s_cand[2].ssid, "fb2");
}

static void ban(const char *ssid, uint32_t now)
{
    for (int i = 0; i < WM_AUTH_FAIL_THRESHOLD; i++)
    {
        wm_select_on_auth_fail(&s_st, ssid, now);
    }
}

void test_scan_prefers_primary(void)
{
    char present[2][WM_SSID_LEN] = { "fb1", "primary" };

    reset();
    TEST_ASSERT_EQUAL_INT(0, wm_select_from_scan(&s_st, s_cand, 3,
                                                 present, 2, 0));
}

void test_scan_falls_back_in_priority_order(void)
{
    char present[2][WM_SSID_LEN] = { "fb2", "fb1" };

    reset();
    /* primary absent -> fb1 (index 1) wins over fb2 despite scan order */
    TEST_ASSERT_EQUAL_INT(1, wm_select_from_scan(&s_st, s_cand, 3,
                                                 present, 2, 0));
}

void test_scan_skips_banned_candidate(void)
{
    char present[2][WM_SSID_LEN] = { "primary", "fb1" };

    reset();
    ban("primary", 1000);
    TEST_ASSERT_EQUAL_INT(1, wm_select_from_scan(&s_st, s_cand, 3,
                                                 present, 2, 1000));
}

void test_scan_all_banned_throttled_retry(void)
{
    char present[2][WM_SSID_LEN] = { "primary", "fb1" };

    reset();
    ban("primary", 1000);
    ban("fb1", 1000);

    /* first banned-only attempt: allowed (highest priority wins) */
    TEST_ASSERT_EQUAL_INT(0, wm_select_from_scan(&s_st, s_cand, 3,
                                                 present, 2, 1000));
    /* within the throttle window: deferred — no 5 s hammering */
    TEST_ASSERT_EQUAL_INT(-1, wm_select_from_scan(&s_st, s_cand, 3,
                                                  present, 2, 2000));
    TEST_ASSERT_EQUAL_INT(-1, wm_select_from_scan(
                                  &s_st, s_cand, 3, present, 2,
                                  1000 + WM_BANNED_RETRY_MS - 1));
    /* throttle elapsed: retried again (meatpi: same SSID may carry a
     * different password elsewhere — never stop trying entirely) */
    TEST_ASSERT_EQUAL_INT(0, wm_select_from_scan(
                                 &s_st, s_cand, 3, present, 2,
                                 1000 + WM_BANNED_RETRY_MS));
}

void test_scan_unbanned_alternative_beats_throttle(void)
{
    char both[2][WM_SSID_LEN] = { "primary", "fb1" };

    reset();
    ban("primary", 1000);

    /* an un-banned alternative is ALWAYS preferred over a banned one,
     * and the throttle never delays it */
    for (uint32_t t = 1000; t < 1000 + 3 * WM_BANNED_RETRY_MS;
         t += 5000)
    {
        TEST_ASSERT_EQUAL_INT(1, wm_select_from_scan(&s_st, s_cand, 3,
                                                     both, 2, t));
    }
}

void test_scan_nothing_present_returns_none(void)
{
    char present[1][WM_SSID_LEN] = { "someone-elses-ap" };

    reset();
    TEST_ASSERT_EQUAL_INT(-1, wm_select_from_scan(&s_st, s_cand, 3,
                                                  present, 1, 0));
}

void test_ban_after_threshold_and_expiry(void)
{
    reset();

    wm_select_on_auth_fail(&s_st, "primary", 0);
    wm_select_on_auth_fail(&s_st, "primary", 0);
    TEST_ASSERT_FALSE(wm_select_is_banned(&s_st, "primary", 0));

    wm_select_on_auth_fail(&s_st, "primary", 0); /* third strike */
    TEST_ASSERT_TRUE(wm_select_is_banned(&s_st, "primary", 0));
    TEST_ASSERT_TRUE(wm_select_is_banned(&s_st, "primary",
                                         WM_BAN_DURATION_MS - 1));
    TEST_ASSERT_FALSE(wm_select_is_banned(&s_st, "primary",
                                          WM_BAN_DURATION_MS + 1));
}

void test_success_clears_failures(void)
{
    reset();
    ban("primary", 0);
    TEST_ASSERT_TRUE(wm_select_is_banned(&s_st, "primary", 0));

    wm_select_on_success(&s_st, "primary");
    TEST_ASSERT_FALSE(wm_select_is_banned(&s_st, "primary", 0));
}

void test_already_banned_does_not_extend(void)
{
    reset();
    ban("primary", 0);

    /* failures during the ban must not push banned_until further out */
    wm_select_on_auth_fail(&s_st, "primary", WM_BAN_DURATION_MS / 2);
    TEST_ASSERT_FALSE(wm_select_is_banned(&s_st, "primary",
                                          WM_BAN_DURATION_MS + 1));
}

void test_sequential_rotates_and_wraps(void)
{
    reset();
    TEST_ASSERT_EQUAL_INT(0, wm_select_sequential(&s_st, s_cand, 3, 0));
    TEST_ASSERT_EQUAL_INT(1, wm_select_sequential(&s_st, s_cand, 3, 0));
    TEST_ASSERT_EQUAL_INT(2, wm_select_sequential(&s_st, s_cand, 3, 0));
    TEST_ASSERT_EQUAL_INT(0, wm_select_sequential(&s_st, s_cand, 3, 0));
}

void test_sequential_skips_banned(void)
{
    reset();
    ban("fb1", 0); /* index 1 */
    TEST_ASSERT_EQUAL_INT(0, wm_select_sequential(&s_st, s_cand, 3, 0));
    TEST_ASSERT_EQUAL_INT(2, wm_select_sequential(&s_st, s_cand, 3, 0));
}

void test_sequential_all_banned_throttled(void)
{
    reset();
    ban("primary", 1000);
    ban("fb1", 1000);
    ban("fb2", 1000);

    /* first banned-only attempt allowed... */
    int pick = wm_select_sequential(&s_st, s_cand, 3, 1000);

    TEST_ASSERT_TRUE(pick >= 0 && pick <= 2);
    /* ...then throttled... */
    TEST_ASSERT_EQUAL_INT(-1, wm_select_sequential(&s_st, s_cand, 3,
                                                   6000));
    /* ...and allowed again after WM_BANNED_RETRY_MS */
    pick = wm_select_sequential(&s_st, s_cand, 3,
                                1000 + WM_BANNED_RETRY_MS);
    TEST_ASSERT_TRUE(pick >= 0 && pick <= 2);
}

void test_sequential_first_pick_is_primary(void)
{
    reset();
    /* the pre-fix cursor made the first blind pick candidate 1 */
    TEST_ASSERT_EQUAL_INT(0, wm_select_sequential(&s_st, s_cand, 3, 0));
}

void test_single_network_banned_trickle(void)
{
    /* THE drive-home case (meatpi 2026-07-08): one configured SSID,
     * wrong password at the current location -> banned; it must keep
     * being retried at the trickle cadence, not go silent for the
     * whole ban window */
    reset();
    ban("primary", 1000);

    TEST_ASSERT_EQUAL_INT(0, wm_select_sequential(&s_st, s_cand, 1,
                                                  1000));
    TEST_ASSERT_EQUAL_INT(-1, wm_select_sequential(&s_st, s_cand, 1,
                                                   6000));
    TEST_ASSERT_EQUAL_INT(0, wm_select_sequential(
                                 &s_st, s_cand, 1,
                                 1000 + WM_BANNED_RETRY_MS));
    TEST_ASSERT_EQUAL_INT(-1, wm_select_sequential(
                                  &s_st, s_cand, 1,
                                  1000 + WM_BANNED_RETRY_MS + 5000));
}

void test_roam_better_available(void)
{
    char home_visible[2][WM_SSID_LEN] = { "primary", "fb2" };
    char only_current[1][WM_SSID_LEN] = { "fb2" };

    reset();

    /* connected to fb2 (index 2); home appears -> migrate to index 0 */
    TEST_ASSERT_EQUAL_INT(0, wm_select_better(&s_st, s_cand, 3, 2,
                                              home_visible, 2, 0));
    /* only the current network visible -> stay */
    TEST_ASSERT_EQUAL_INT(-1, wm_select_better(&s_st, s_cand, 3, 2,
                                               only_current, 1, 0));
    /* already on the primary -> never roam */
    TEST_ASSERT_EQUAL_INT(-1, wm_select_better(&s_st, s_cand, 3, 0,
                                               home_visible, 2, 0));
    /* the better network is banned -> stay on the working fallback */
    ban("primary", 0);
    TEST_ASSERT_EQUAL_INT(-1, wm_select_better(&s_st, s_cand, 3, 2,
                                               home_visible, 2, 0));
    /* ban expired -> migrate again */
    TEST_ASSERT_EQUAL_INT(0, wm_select_better(&s_st, s_cand, 3, 2,
                                              home_visible, 2,
                                              WM_BAN_DURATION_MS + 1));
}

void test_duplicate_ssid_ban_covers_both(void)
{
    /* KNOWN behavior: bans are keyed by SSID string — two candidates
     * sharing a name (same SSID, different passwords) ban together.
     * Pinned here so a future per-candidate ban is a conscious change. */
    char present[1][WM_SSID_LEN] = { "twin" };
    wm_network_t twins[2];

    memset(twins, 0, sizeof(twins));
    strcpy(twins[0].ssid, "twin");
    strcpy(twins[0].password, "pw-home");
    strcpy(twins[1].ssid, "twin");
    strcpy(twins[1].password, "pw-office");

    reset();
    ban("twin", 1000);
    /* both entries banned -> only the throttled retry path returns one */
    TEST_ASSERT_EQUAL_INT(0, wm_select_from_scan(&s_st, twins, 2,
                                                 present, 1, 1000));
    TEST_ASSERT_EQUAL_INT(-1, wm_select_from_scan(&s_st, twins, 2,
                                                  present, 1, 2000));
}

void test_parse_ap_ipv4(void)
{
    uint32_t ip = 0;

    /* the legacy default parses to host-order 0xC0A85001 */
    TEST_ASSERT_TRUE(wm_parse_ap_ipv4("192.168.80.1", &ip));
    TEST_ASSERT_EQUAL_HEX32(0xC0A85001u, ip);

    TEST_ASSERT_TRUE(wm_parse_ap_ipv4("10.0.0.1", &ip));
    TEST_ASSERT_EQUAL_HEX32(0x0A000001u, ip);
    TEST_ASSERT_TRUE(wm_parse_ap_ipv4("172.16.5.254", &ip));

    /* rejects: host octet 0/255, out-of-range octet, short/long, junk,
       empty, NULL, leading/trailing garbage, negative */
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("192.168.80.0", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("192.168.80.255", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("192.168.80.256", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("192.168.80", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("192.168.80.1.5", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("192.168.80.1 ", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("192.168..1", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("garbage", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4("-1.0.0.1", &ip));
    TEST_ASSERT_FALSE(wm_parse_ap_ipv4(NULL, &ip));
}

void test_parse_ipv4_plain(void)
{
    uint32_t ip = 0;

    /* the plain parser accepts network/broadcast host octets (it feeds
       netmask + DNS fields, not host addresses) */
    TEST_ASSERT_TRUE(wm_parse_ipv4("255.255.255.0", &ip));
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFF00u, ip);
    TEST_ASSERT_TRUE(wm_parse_ipv4("10.0.0.0", &ip));
    TEST_ASSERT_TRUE(wm_parse_ipv4("1.1.1.1", &ip));
    TEST_ASSERT_EQUAL_HEX32(0x01010101u, ip);

    /* same lexical rules as the host-address parser */
    TEST_ASSERT_FALSE(wm_parse_ipv4("256.0.0.1", &ip));
    TEST_ASSERT_FALSE(wm_parse_ipv4("10.0.0", &ip));
    TEST_ASSERT_FALSE(wm_parse_ipv4("10.0.0.1.2", &ip));
    TEST_ASSERT_FALSE(wm_parse_ipv4("", &ip));
    TEST_ASSERT_FALSE(wm_parse_ipv4(NULL, &ip));
}

void test_netmask_valid(void)
{
    /* contiguous masks /8../31 */
    TEST_ASSERT_TRUE(wm_netmask_valid(0xFF000000u));  /* /8  */
    TEST_ASSERT_TRUE(wm_netmask_valid(0xFFFF0000u));  /* /16 */
    TEST_ASSERT_TRUE(wm_netmask_valid(0xFFFFFF00u));  /* /24 */
    TEST_ASSERT_TRUE(wm_netmask_valid(0xFFFFFF80u));  /* /25 */
    TEST_ASSERT_TRUE(wm_netmask_valid(0xFFFFFFFEu));  /* /31 */

    /* degenerate or holed masks */
    TEST_ASSERT_FALSE(wm_netmask_valid(0));            /* /0        */
    TEST_ASSERT_FALSE(wm_netmask_valid(0xFFFFFFFFu));  /* /32       */
    TEST_ASSERT_FALSE(wm_netmask_valid(0xFF00FF00u));  /* holed     */
    TEST_ASSERT_FALSE(wm_netmask_valid(0x80000001u));  /* holed     */
    TEST_ASSERT_FALSE(wm_netmask_valid(0x00FFFF00u));  /* not MSB   */
}

void test_backoff_curve(void)
{
    /* first two attempts stay on the 5 s cadence (candidate walk) */
    TEST_ASSERT_EQUAL_INT(0, wm_backoff_skip_loops(1));
    TEST_ASSERT_EQUAL_INT(0, wm_backoff_skip_loops(2));

    /* then double: 10 / 20 s ... */
    TEST_ASSERT_EQUAL_INT(1, wm_backoff_skip_loops(3));
    TEST_ASSERT_EQUAL_INT(3, wm_backoff_skip_loops(4));

    /* ... capped at the 30 s cadence forever after */
    TEST_ASSERT_EQUAL_INT(WM_BACKOFF_MAX_SKIP_LOOPS,
                          wm_backoff_skip_loops(5));
    TEST_ASSERT_EQUAL_INT(WM_BACKOFF_MAX_SKIP_LOOPS,
                          wm_backoff_skip_loops(6));
    TEST_ASSERT_EQUAL_INT(WM_BACKOFF_MAX_SKIP_LOOPS,
                          wm_backoff_skip_loops(7));
    TEST_ASSERT_EQUAL_INT(WM_BACKOFF_MAX_SKIP_LOOPS,
                          wm_backoff_skip_loops(1000));

    /* degenerate inputs never go negative */
    TEST_ASSERT_EQUAL_INT(0, wm_backoff_skip_loops(0));
    TEST_ASSERT_EQUAL_INT(0, wm_backoff_skip_loops(-5));
}

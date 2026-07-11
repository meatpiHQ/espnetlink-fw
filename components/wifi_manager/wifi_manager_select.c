/**
 * @file wifi_manager_select.c
 * @brief Pure STA candidate selection + auth-failure ban list.
 *        No IDF dependencies — compiled as-is by the host unit tests.
 *        Time comes in as a millisecond tick from the caller.
 */
#include "wifi_manager_private.h"

#include <stdlib.h>
#include <string.h>

bool wm_parse_ipv4(const char *s, uint32_t *out)
{
    if (s == NULL || out == NULL)
    {
        return false;
    }

    uint32_t octet[4];

    for (int i = 0; i < 4; i++)
    {
        if (*s < '0' || *s > '9') /* reject empty / leading sign / space */
        {
            return false;
        }

        char *end = NULL;
        long v = strtol(s, &end, 10);

        if (end == s || v < 0 || v > 255)
        {
            return false;
        }

        octet[i] = (uint32_t)v;
        s = end;

        if (i < 3)
        {
            if (*s != '.')
            {
                return false;
            }
            s++;
        }
    }

    if (*s != '\0')
    {
        return false; /* trailing junk */
    }

    *out = (octet[0] << 24) | (octet[1] << 16) | (octet[2] << 8) | octet[3];
    return true;
}

bool wm_parse_ap_ipv4(const char *s, uint32_t *out)
{
    uint32_t v;

    if (!wm_parse_ipv4(s, &v))
    {
        return false;
    }

    /* a /24 host octet of 0 (network) or 255 (broadcast) is not a usable
       host/gateway address */
    if ((v & 0xFFu) == 0 || (v & 0xFFu) == 255)
    {
        return false;
    }

    *out = v;
    return true;
}

bool wm_netmask_valid(uint32_t mask)
{
    if (mask == 0 || mask == 0xFFFFFFFFu)
    {
        return false;
    }

    /* contiguous ones from the MSB: the inverse must be 2^n - 1 */
    uint32_t inv = ~mask;

    return ((inv + 1u) & inv) == 0;
}

void wm_select_init(wm_select_state_t *st)
{
    memset(st, 0, sizeof(*st));
    /* -1 = "nothing returned yet" so the first sequential pick is the
     * PRIMARY (a zeroed cursor made the first rotation start at [1]) */
    st->seq_cursor = -1;
}

static wm_ban_entry_t *find_entry(wm_select_state_t *st, const char *ssid)
{
    for (size_t i = 0; i < sizeof(st->bans) / sizeof(st->bans[0]); i++)
    {
        if (st->bans[i].ssid[0] != '\0' &&
            strcmp(st->bans[i].ssid, ssid) == 0)
        {
            return &st->bans[i];
        }
    }

    return NULL;
}

static wm_ban_entry_t *find_or_create(wm_select_state_t *st, const char *ssid)
{
    wm_ban_entry_t *e = find_entry(st, ssid);

    if (e != NULL)
    {
        return e;
    }

    for (size_t i = 0; i < sizeof(st->bans) / sizeof(st->bans[0]); i++)
    {
        if (st->bans[i].ssid[0] == '\0')
        {
            e = &st->bans[i];
            strncpy(e->ssid, ssid, WM_SSID_LEN - 1);
            e->ssid[WM_SSID_LEN - 1] = '\0';
            e->fail_count = 0;
            e->banned_until_ms = 0;
            return e;
        }
    }

    return NULL; /* table full: worst case we never ban this SSID */
}

bool wm_select_is_banned(const wm_select_state_t *st, const char *ssid,
                         uint32_t now_ms)
{
    if (ssid == NULL || ssid[0] == '\0')
    {
        return false;
    }

    for (size_t i = 0; i < sizeof(st->bans) / sizeof(st->bans[0]); i++)
    {
        const wm_ban_entry_t *e = &st->bans[i];

        if (e->ssid[0] != '\0' && strcmp(e->ssid, ssid) == 0)
        {
            /* tick-wrap-safe: banned while (banned_until - now) > 0 */
            return e->banned_until_ms != 0 &&
                   (int32_t)(e->banned_until_ms - now_ms) > 0;
        }
    }

    return false;
}

void wm_select_on_auth_fail(wm_select_state_t *st, const char *ssid,
                            uint32_t now_ms)
{
    if (ssid == NULL || ssid[0] == '\0')
    {
        return;
    }

    wm_ban_entry_t *e = find_or_create(st, ssid);

    if (e == NULL || wm_select_is_banned(st, ssid, now_ms))
    {
        return; /* already banned: don't extend the window */
    }

    e->fail_count++;

    if (e->fail_count >= WM_AUTH_FAIL_THRESHOLD)
    {
        e->fail_count = 0;
        e->banned_until_ms = now_ms + WM_BAN_DURATION_MS;

        if (e->banned_until_ms == 0) /* avoid the "not banned" sentinel */
        {
            e->banned_until_ms = 1;
        }
    }
}

void wm_select_on_success(wm_select_state_t *st, const char *ssid)
{
    if (ssid == NULL || ssid[0] == '\0')
    {
        return;
    }

    wm_ban_entry_t *e = find_entry(st, ssid);

    if (e != NULL)
    {
        e->fail_count = 0;
        e->banned_until_ms = 0;
    }
}

static bool ssid_present(const char (*present)[WM_SSID_LEN],
                         size_t present_count, const char *ssid)
{
    if (ssid == NULL || ssid[0] == '\0')
    {
        return false;
    }

    for (size_t i = 0; i < present_count; i++)
    {
        if (strcmp(present[i], ssid) == 0)
        {
            return true;
        }
    }

    return false;
}

/** Banned-only fallback throttle (2026-07-08, meatpi): a banned network
 *  must still be RETRIED when it's all we have — the same SSID name can
 *  carry a different password at another location ("coffee shop" case:
 *  auth fails away from home, must reconnect promptly back home). But
 *  never at the full reconnect cadence (the old code hammered it every
 *  cycle). One banned attempt per WM_BANNED_RETRY_MS is the compromise. */
static bool banned_retry_due(wm_select_state_t *st, uint32_t now_ms)
{
    return st->last_banned_try_ms == 0 ||
           (int32_t)(now_ms - st->last_banned_try_ms) >=
               (int32_t)WM_BANNED_RETRY_MS;
}

static void note_banned_try(wm_select_state_t *st, uint32_t now_ms)
{
    st->last_banned_try_ms = (now_ms != 0) ? now_ms : 1;
}

int wm_select_from_scan(wm_select_state_t *st, const wm_network_t *cand,
                        size_t cand_count,
                        const char (*present)[WM_SSID_LEN],
                        size_t present_count, uint32_t now_ms)
{
    /* first pass: priority order, un-banned + visible */
    for (size_t i = 0; i < cand_count; i++)
    {
        if (ssid_present(present, present_count, cand[i].ssid) &&
            !wm_select_is_banned(st, cand[i].ssid, now_ms))
        {
            return (int)i;
        }
    }

    /* everything visible is banned: throttled retry (see above) */
    for (size_t i = 0; i < cand_count; i++)
    {
        if (ssid_present(present, present_count, cand[i].ssid))
        {
            if (!banned_retry_due(st, now_ms))
            {
                return -1;
            }

            note_banned_try(st, now_ms);
            return (int)i;
        }
    }

    return -1;
}

int wm_select_sequential(wm_select_state_t *st, const wm_network_t *cand,
                         size_t cand_count, uint32_t now_ms)
{
    if (cand_count == 0)
    {
        return -1;
    }

    /* rotate, skipping banned candidates */
    for (size_t i = 0; i < cand_count; i++)
    {
        st->seq_cursor = (st->seq_cursor + 1) % (int)cand_count;

        if (!wm_select_is_banned(st, cand[st->seq_cursor].ssid, now_ms))
        {
            return st->seq_cursor;
        }
    }

    /* all banned: throttled retry (same policy as the scan path) */
    if (!banned_retry_due(st, now_ms))
    {
        return -1;
    }

    note_banned_try(st, now_ms);
    st->seq_cursor = (st->seq_cursor + 1) % (int)cand_count;
    return st->seq_cursor;
}

int wm_select_better(wm_select_state_t *st, const wm_network_t *cand,
                     size_t cand_count, int current_idx,
                     const char (*present)[WM_SSID_LEN],
                     size_t present_count, uint32_t now_ms)
{
    if (current_idx <= 0 || (size_t)current_idx > cand_count)
    {
        return -1;              /* already on the primary (or invalid)   */
    }

    for (int i = 0; i < current_idx; i++)
    {
        if (ssid_present(present, present_count, cand[i].ssid) &&
            !wm_select_is_banned(st, cand[i].ssid, now_ms))
        {
            return i;
        }
    }

    return -1;
}

int wm_backoff_skip_loops(int32_t retry_count)
{
    if (retry_count <= 2)
    {
        return 0;                       /* keep the candidate walk fast */
    }

    int shift = retry_count - 2;        /* 3rd failure -> 1 loop (10 s) */

    if (shift > 4)
    {
        shift = 4;
    }

    int skip = (1 << shift) - 1;        /* 1, 3, 7, 15 loops            */

    return (skip > WM_BACKOFF_MAX_SKIP_LOOPS) ? WM_BACKOFF_MAX_SKIP_LOOPS
                                              : skip;
}

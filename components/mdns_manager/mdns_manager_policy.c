/**
 * @file mdns_manager_policy.c
 * @brief PURE string builders for the mDNS contract — host-testable.
 */
#include <stdio.h>

#include "mdns_manager_private.h"

bool mm_format_mac(const uint8_t mac[6], char *out, size_t cap)
{
    if (mac == NULL || out == NULL)
    {
        return false;
    }

    return snprintf(out, cap, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
                    mac[1], mac[2], mac[3], mac[4],
                    mac[5]) < (int)cap;
}

bool mm_build_hostname(const char *id, char *out, size_t cap)
{
    if (id == NULL || id[0] == '\0' || out == NULL)
    {
        return false;
    }

    return snprintf(out, cap, "espnetlink_%s", id) < (int)cap;
}

bool mm_build_hostname_local(const char *id, char *out, size_t cap)
{
    if (id == NULL || id[0] == '\0' || out == NULL)
    {
        return false;
    }

    return snprintf(out, cap, "espnetlink_%s.local", id) < (int)cap;
}

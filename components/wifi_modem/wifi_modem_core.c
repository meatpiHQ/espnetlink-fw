/**
 * @file wifi_modem_core.c
 * @brief Pure helpers of wifi_modem (no IDF services, host-tested, §7):
 *        usb_mode string mapping, the USB presentation and runtime-switch
 *        policies, the AP password policy (stock check, generator,
 *        provisioning decision), resolver pick, IPv4 formatting.
 */
#include "wifi_modem_private.h"

#include <stdio.h>
#include <string.h>

const char *wmc_usb_mode_str(wifi_modem_usb_mode_t mode)
{
    switch (mode)
    {
        case WIFI_MODEM_USB_AUTO:
            return "auto";
        case WIFI_MODEM_USB_NCM:
            return "ncm";
        default:
            return "console";
    }
}

bool wmc_usb_mode_parse(const char *s, wifi_modem_usb_mode_t *out)
{
    if (s == NULL || out == NULL)
    {
        return false;
    }

    if (strcmp(s, "auto") == 0)
    {
        *out = WIFI_MODEM_USB_AUTO;
        return true;
    }

    if (strcmp(s, "ncm") == 0)
    {
        *out = WIFI_MODEM_USB_NCM;
        return true;
    }

    if (strcmp(s, "console") == 0)
    {
        *out = WIFI_MODEM_USB_CONSOLE;
        return true;
    }

    return false;
}

bool wmc_usb_ethernet_present(wifi_modem_usb_mode_t mode)
{
    return mode != WIFI_MODEM_USB_CONSOLE;
}

bool wmc_usb_data_switchable(wifi_modem_usb_mode_t mode)
{
    return mode != WIFI_MODEM_USB_NCM;
}

bool wmc_credentials_allowed(uint32_t local_be, uint32_t ap_be, uint32_t sta_be)
{
    if (local_be == 0)
    {
        return false;
    }

    if (ap_be != 0 && local_be == ap_be)
    {
        return false;
    }

    if (sta_be != 0 && local_be == sta_be)
    {
        return false;
    }

    return true;
}

bool wmc_password_is_stock(const char *password)
{
    return password == NULL || password[0] == '\0' ||
           strcmp(password, WIFI_MODEM_STOCK_AP_PASSWORD) == 0;
}

bool wmc_generate_password(char *out, size_t out_len, size_t n_chars,
                           uint32_t (*rng)(void))
{
    /* no 0/O, 1/l/I: the password ends up typed from a label */
    static const char ALPHABET[] =
        "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghjkmnpqrstuvwxyz23456789";
    const uint32_t n_alpha = (uint32_t)(sizeof(ALPHABET) - 1);

    if (out == NULL || rng == NULL || n_chars == 0 || out_len < n_chars + 1)
    {
        return false;
    }

    for (size_t i = 0; i < n_chars; i++)
    {
        out[i] = ALPHABET[rng() % n_alpha];
    }
    out[n_chars] = '\0';
    return true;
}

wmc_provision_action_t wmc_provision_decide(bool provisioned,
                                            const char *ap_password)
{
    if (provisioned)
    {
        return WMC_PROVISION_NONE;
    }

    return wmc_password_is_stock(ap_password) ? WMC_PROVISION_GENERATE
                                              : WMC_PROVISION_MARK;
}

uint32_t wmc_pick_upstream_dns(uint32_t learned_be)
{
    if (learned_be != 0)
    {
        return learned_be;
    }

    /* 8.8.8.8 in network order, written byte-wise so the host test is
       endian-agnostic */
    const uint8_t google[4] = { 8, 8, 8, 8 };
    uint32_t v;

    memcpy(&v, google, sizeof(v));
    return v;
}

bool wmc_format_ipv4(uint32_t addr_be, char *out, size_t out_len)
{
    uint8_t b[4];

    if (out == NULL || out_len < 16)
    {
        return false;
    }

    memcpy(b, &addr_be, sizeof(b));
    snprintf(out, out_len, "%u.%u.%u.%u", (unsigned)b[0], (unsigned)b[1],
             (unsigned)b[2], (unsigned)b[3]);
    return true;
}

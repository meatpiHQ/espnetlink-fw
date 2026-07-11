/**
 * @file vpn_manager_check.c
 * @brief Pure config validation (host-tested; no esp deps beyond
 *        esp_err.h). The settings on_validate speaks through
 *        vpn_check_config so the UI gets a reason, not just a 400.
 */
#include <string.h>

#include "vpn_manager_private.h"

#define WG_KEY_B64_LEN 44 /* base64(32 bytes) incl. the trailing '=' */

static bool b64_char(char c)
{
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') || c == '+' || c == '/';
}

bool vpn_check_wg_key(const char *key_b64)
{
    if (key_b64 == NULL || strlen(key_b64) != WG_KEY_B64_LEN)
    {
        return false;
    }

    for (int i = 0; i < WG_KEY_B64_LEN - 1; i++)
    {
        if (!b64_char(key_b64[i]))
        {
            return false;
        }
    }

    return key_b64[WG_KEY_B64_LEN - 1] == '=';
}

bool vpn_check_endpoint(const char *host)
{
    if (host == NULL)
    {
        return false;
    }

    size_t len = strlen(host);

    if (len < 1 || len > 63)
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        char c = host[i];
        bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') || c == '.' || c == '-' ||
                  c == ':' || c == '_';

        if (!ok)
        {
            return false;
        }
    }

    return true;
}

const char *vpn_check_config(const vpn_config_t *cfg)
{
    if (!cfg->enabled)
    {
        return NULL; /* disabled config is always storable */
    }

    if (cfg->tailscale)
    {
        size_t len = strlen(cfg->ts_auth_key);

        /* tskey-auth-... (Tailscale) or a headscale preauth key (hex);
         * shape-check only — the coordinator is the judge */
        if (len < 16)
        {
            return "ts_auth_key is required for tailscale (from your "
                   "coordinator's admin)";
        }

        if (cfg->ts_control_url[0] != '\0' &&
            !vpn_check_endpoint(cfg->ts_control_url))
        {
            return "ts_control_url must be a hostname or IP (no "
                   "scheme)";
        }

        return NULL; /* wireguard fields not required for tailscale */
    }

    if (!vpn_check_wg_key(cfg->private_key))
    {
        return "private_key must be a 44-char base64 WireGuard key "
               "(use the keygen)";
    }

    if (!vpn_check_wg_key(cfg->peer_public_key))
    {
        return "peer_public_key must be a 44-char base64 WireGuard key";
    }

    if (cfg->preshared_key[0] != '\0' &&
        !vpn_check_wg_key(cfg->preshared_key))
    {
        return "preshared_key must be empty or a 44-char base64 key";
    }

    if (!vpn_check_endpoint(cfg->endpoint))
    {
        return "endpoint must be a hostname or IP";
    }

    if (cfg->port < 1 || cfg->port > 65535)
    {
        return "port must be 1..65535";
    }

    if (cfg->address[0] == '\0')
    {
        return "address (local tunnel IP) is required";
    }

    return NULL;
}

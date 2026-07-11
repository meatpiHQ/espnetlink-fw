/**
 * @file vpn_manager_wg.c
 * @brief esp_wireguard glue (legacy vpn_wireguard.c semantics kept):
 *        endpoint resolved to IPv4 once per up (the port doesn't
 *        resolve internally), AllowedIPs=0.0.0.0 falls back to the
 *        interface address /32 (esp_wireguard uses allowed_ip as the
 *        LOCAL tunnel IP), peer-up = the wireguardif poll. Called from
 *        the state task ONLY.
 */
#include <string.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wireguard.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "vpn_manager_private.h"

static const char *TAG = "vpn_manager";

static wireguard_ctx_t s_ctx;
static wireguard_config_t s_cfg;
/* esp_wireguard keeps POINTERS into its config — these back them */
static char s_addr[32];
static char s_endpoint[64];
static char s_priv[64];
static char s_pub[64];
static char s_psk[64];
static char s_allow[32];
static char s_mask[32];

static bool resolve_ipv4(const char *host, char *out, size_t len)
{
    struct in_addr a;

    if (inet_pton(AF_INET, host, &a) == 1)
    {
        strlcpy(out, host, len);
        return true;
    }

    struct addrinfo hints = { .ai_family = AF_INET,
                              .ai_socktype = SOCK_DGRAM };
    struct addrinfo *res = NULL;

    if (getaddrinfo(host, NULL, &hints, &res) != 0 || res == NULL)
    {
        return false;
    }

    bool ok = false;

    for (struct addrinfo *ai = res; ai != NULL; ai = ai->ai_next)
    {
        if (ai->ai_family == AF_INET && ai->ai_addr != NULL)
        {
            struct sockaddr_in *sa = (struct sockaddr_in *)ai->ai_addr;

            ok = inet_ntop(AF_INET, &sa->sin_addr, out, len) != NULL;
            break;
        }
    }

    freeaddrinfo(res);
    return ok;
}

esp_err_t vpn_wg_up(const vpn_config_t *cfg)
{
    if (s_ctx.config != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_cfg, 0, sizeof(s_cfg));
    strlcpy(s_priv, cfg->private_key, sizeof(s_priv));
    strlcpy(s_pub, cfg->peer_public_key, sizeof(s_pub));
    s_cfg.private_key = s_priv;
    s_cfg.public_key = s_pub;

    if (cfg->preshared_key[0] != '\0')
    {
        strlcpy(s_psk, cfg->preshared_key, sizeof(s_psk));
        s_cfg.preshared_key = s_psk;
    }

    /* esp_wireguard's allowed_ip is the LOCAL tunnel IP; 0.0.0.0 (the
     * route-everything idiom) falls back to Interface Address /32 */
    if (cfg->allowed_ip[0] != '\0' &&
        strcmp(cfg->allowed_ip, "0.0.0.0") != 0)
    {
        strlcpy(s_allow, cfg->allowed_ip, sizeof(s_allow));
        s_cfg.allowed_ip = s_allow;

        if (cfg->allowed_ip_mask[0] != '\0')
        {
            strlcpy(s_mask, cfg->allowed_ip_mask, sizeof(s_mask));
            s_cfg.allowed_ip_mask = s_mask;
        }
    }
    else
    {
        strlcpy(s_addr, cfg->address, sizeof(s_addr));

        char *slash = strchr(s_addr, '/');

        if (slash != NULL)
        {
            *slash = '\0';
        }

        s_cfg.allowed_ip = s_addr;
        s_cfg.allowed_ip_mask = "255.255.255.255";
        ESP_LOGI(TAG, "local WG IP from address: %s/32", s_addr);
    }

    if (!resolve_ipv4(cfg->endpoint, s_endpoint, sizeof(s_endpoint)))
    {
        ESP_LOGW(TAG, "endpoint '%s' did not resolve; using as-is",
                 cfg->endpoint);
        strlcpy(s_endpoint, cfg->endpoint, sizeof(s_endpoint));
    }

    s_cfg.endpoint = s_endpoint;
    s_cfg.port = cfg->port;
    s_cfg.persistent_keepalive = cfg->keepalive_s;
    s_cfg.listen_port = 0;
    s_cfg.fw_mark = 0;

    esp_err_t err = esp_wireguard_init(&s_cfg, &s_ctx);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "wireguard init: %s", esp_err_to_name(err));
        memset(&s_ctx, 0, sizeof(s_ctx));
        return err;
    }

    err = esp_wireguard_connect(&s_ctx);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "wireguard connect: %s", esp_err_to_name(err));
        vpn_wg_down();
        return err;
    }

    ESP_LOGI(TAG, "tunnel up to %s:%d (handshake pending)", s_endpoint,
             cfg->port);
    return ESP_OK;
}

void vpn_wg_down(void)
{
    if (s_ctx.config == NULL)
    {
        return;
    }

    if (s_ctx.netif != NULL)
    {
        esp_err_t err = esp_wireguard_disconnect(&s_ctx);

        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "disconnect: %s", esp_err_to_name(err));
        }
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    memset(&s_cfg, 0, sizeof(s_cfg));
    memset(s_priv, 0, sizeof(s_priv)); /* don't leave key material */
    memset(s_psk, 0, sizeof(s_psk));
    ESP_LOGI(TAG, "tunnel down");
}

bool vpn_wg_peer_up(void)
{
    if (s_ctx.config == NULL || s_ctx.netif == NULL)
    {
        return false;
    }

    return esp_wireguardif_peer_is_up(&s_ctx) == ESP_OK;
}

esp_err_t vpn_wg_set_default_route(void)
{
    if (s_ctx.config == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_wireguard_set_default(&s_ctx);
}

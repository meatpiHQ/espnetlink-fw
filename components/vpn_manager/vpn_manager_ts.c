/**
 * @file vpn_manager_ts.c
 * @brief Tailscale glue (vendored microlink) — the `type = tailscale`
 *        path of the state task. Everything microlink allocates lives
 *        only between vpn_ts_up() and vpn_ts_down(): zero memory cost
 *        while tailscale is disabled or wireguard is selected.
 *        Called from the state task ONLY.
 */
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "microlink.h"

#include "vpn_manager_private.h"

static const char *TAG = "vpn_manager";

static microlink_t *s_ml;
/* microlink keeps POINTERS into its config — these back them */
static char s_auth_key[96];
static char s_device_name[48];
static char s_control_url[64];

esp_err_t vpn_ts_up(const vpn_config_t *cfg)
{
    if (s_ml != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    strlcpy(s_auth_key, cfg->ts_auth_key, sizeof(s_auth_key));
    strlcpy(s_device_name, cfg->ts_device_name, sizeof(s_device_name));
    strlcpy(s_control_url, cfg->ts_control_url, sizeof(s_control_url));

    microlink_config_t ml_cfg =
    {
        .auth_key = s_auth_key,
        .device_name = s_device_name[0] != '\0'
            ? s_device_name : microlink_default_device_name(),
        .enable_derp = true,
        .enable_stun = true,
        .enable_disco = true,
        .max_peers = 0, /* microlink default */
        .control_url = s_control_url[0] != '\0' ? s_control_url : NULL,
    };

    s_ml = microlink_init(&ml_cfg);

    if (s_ml == NULL)
    {
        ESP_LOGE(TAG, "microlink init failed");
        return ESP_FAIL;
    }

    esp_err_t err = microlink_start(s_ml);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "microlink start: %s", esp_err_to_name(err));
        vpn_ts_down();
        return err;
    }

    ESP_LOGI(TAG, "tailscale starting (%s)",
             s_control_url[0] ? s_control_url : "tailscale.com");
    return ESP_OK;
}

void vpn_ts_down(void)
{
    if (s_ml == NULL)
    {
        return;
    }

    (void)microlink_stop(s_ml);
    microlink_destroy(s_ml); /* frees ALL microlink memory */
    s_ml = NULL;
    memset(s_auth_key, 0, sizeof(s_auth_key)); /* key material */
    ESP_LOGI(TAG, "tailscale down");
}

bool vpn_ts_connected(void)
{
    return s_ml != NULL && microlink_is_connected(s_ml);
}

bool vpn_ts_failed(void)
{
    return s_ml != NULL && microlink_get_state(s_ml) == ML_STATE_ERROR;
}

void vpn_ts_status(char *ip, size_t ip_len, int *peers)
{
    ip[0] = '\0';
    *peers = 0;

    if (s_ml == NULL)
    {
        return;
    }

    uint32_t vpn_ip = microlink_get_vpn_ip(s_ml);

    if (vpn_ip != 0 && ip_len >= 16)
    {
        microlink_ip_to_str(vpn_ip, ip);
    }

    *peers = microlink_get_peer_count(s_ml);
}

/**
 * @file vpn_manager_keygen.c
 * @brief Device-side Curve25519 keypair generation (legacy
 *        vpn_keygen.c semantics: 32 random bytes, x25519_base clamps
 *        internally like upstream WireGuard). The private key goes
 *        STRAIGHT into pending settings via settings_manager_set —
 *        it never travels over HTTP; the caller gets only the public
 *        key.
 */
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "mbedtls/base64.h"

#include "settings_manager.h"
#include "x25519.h"

#include "vpn_manager_private.h"

static const char *TAG = "vpn_manager";

static void wipe(void *v, size_t n)
{
    volatile unsigned char *p = (volatile unsigned char *)v;

    while (n--)
    {
        *p++ = 0;
    }
}

static esp_err_t b64(const unsigned char *in, size_t in_len, char *out,
                     size_t out_size)
{
    size_t olen = 0;

    if (mbedtls_base64_encode((unsigned char *)out, out_size, &olen, in,
                              in_len) != 0 || olen >= out_size)
    {
        return ESP_FAIL;
    }

    out[olen] = '\0';
    return ESP_OK;
}

esp_err_t vpn_manager_keygen(char *public_key_b64, size_t len)
{
    unsigned char sk[32];
    unsigned char pk[32];
    char sk_b64[64];
    esp_err_t err = ESP_FAIL;
    cJSON *settings = NULL;
    char verr[96] = "";

    if (public_key_b64 == NULL || len < 46)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_fill_random(sk, sizeof(sk));

    if (x25519_base(pk, sk, 1) != 0)
    {
        ESP_LOGE(TAG, "x25519_base failed");
        goto out;
    }

    if (b64(sk, sizeof(sk), sk_b64, sizeof(sk_b64)) != ESP_OK ||
        b64(pk, sizeof(pk), public_key_b64, len) != ESP_OK)
    {
        goto out;
    }

    /* persist the private key into pending settings (reboot applies) */
    if (settings_manager_get("vpn_manager", &settings) != ESP_OK ||
        settings == NULL)
    {
        goto out;
    }

    cJSON *item = cJSON_GetObjectItemCaseSensitive(settings,
                                                   "private_key");

    if (item != NULL && cJSON_IsString(item))
    {
        cJSON_SetValuestring(item, sk_b64);
    }
    else
    {
        cJSON_AddStringToObject(settings, "private_key", sk_b64);
    }

    err = settings_manager_set("vpn_manager", settings, verr,
                               sizeof(verr), NULL);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "keygen persist failed: %s", verr);
    }
    else
    {
        ESP_LOGI(TAG, "new WireGuard keypair stored (public %s)",
                 public_key_b64);
    }

out:
    if (settings != NULL)
    {
        cJSON_Delete(settings);
    }

    wipe(sk, sizeof(sk));
    wipe(sk_b64, sizeof(sk_b64));
    wipe(pk, sizeof(pk));

    if (err != ESP_OK)
    {
        public_key_b64[0] = '\0';
    }

    return err;
}

/**
 * @file http_client_manager_auth.c
 * @brief PURE url/auth helpers — own tiny base64, no mbedtls, no RTOS;
 *        host-testable.
 */
#include <stdio.h>
#include <string.h>

#include "http_client_manager_private.h"

bool hc_url_valid(const char *url)
{
    if (url == NULL)
    {
        return false;
    }

    const char *host = NULL;

    if (strncmp(url, "http://", 7) == 0)
    {
        host = url + 7;
    }
    else if (strncmp(url, "https://", 8) == 0)
    {
        host = url + 8;
    }

    return host != NULL && host[0] != '\0' && host[0] != ':' &&
           host[0] != '/';
}

bool hc_url_is_tls(const char *url)
{
    return url != NULL && strncmp(url, "https://", 8) == 0;
}

static const char B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t hc_base64(const uint8_t *in, size_t len, char *out, size_t cap)
{
    size_t need = ((len + 2) / 3) * 4;

    if (in == NULL || out == NULL || need + 1 > cap)
    {
        return 0;
    }

    size_t o = 0;

    for (size_t i = 0; i < len; i += 3)
    {
        uint32_t v = (uint32_t)in[i] << 16;
        size_t rem = len - i;

        if (rem > 1)
        {
            v |= (uint32_t)in[i + 1] << 8;
        }

        if (rem > 2)
        {
            v |= in[i + 2];
        }

        out[o++] = B64[(v >> 18) & 0x3F];
        out[o++] = B64[(v >> 12) & 0x3F];
        out[o++] = (rem > 1) ? B64[(v >> 6) & 0x3F] : '=';
        out[o++] = (rem > 2) ? B64[v & 0x3F] : '=';
    }

    out[o] = '\0';
    return o;
}

bool hc_auth_bearer(const char *token, char *out, size_t cap)
{
    if (token == NULL || token[0] == '\0')
    {
        return false;
    }

    return snprintf(out, cap, "Bearer %s", token) < (int)cap;
}

bool hc_auth_basic(const char *user, const char *pass, char *out,
                   size_t cap)
{
    if (user == NULL || pass == NULL)
    {
        return false;
    }

    char plain[128];
    int n = snprintf(plain, sizeof(plain), "%s:%s", user, pass);

    if (n < 0 || n >= (int)sizeof(plain) || cap < 7)
    {
        return false;
    }

    memcpy(out, "Basic ", 6);
    return hc_base64((const uint8_t *)plain, (size_t)n, out + 6,
                     cap - 6) > 0;
}

const char *hc_header_split(const char *header, char *key, size_t cap)
{
    if (header == NULL || key == NULL)
    {
        return NULL;
    }

    const char *colon = strchr(header, ':');

    if (colon == NULL || colon == header ||
        (size_t)(colon - header) >= cap)
    {
        return NULL;
    }

    memcpy(key, header, (size_t)(colon - header));
    key[colon - header] = '\0';

    const char *value = colon + 1;

    while (*value == ' ')
    {
        value++;
    }

    return (*value != '\0') ? value : NULL;
}

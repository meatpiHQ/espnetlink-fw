/**
 * @file cert_manager_policy.c
 * @brief PURE set-name validation, part mapping, and PEM plausibility —
 *        no filesystem, no RTOS; host-testable.
 */
#include <string.h>

#include "cert_manager_private.h"

bool cm_set_name_valid(const char *name)
{
    if (name == NULL || name[0] == '\0')
    {
        return false;
    }

    size_t len = strlen(name);

    if (len > CERT_MANAGER_NAME_MAX)
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        char c = name[i];

        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
              c == '_' || c == '-'))
        {
            return false;
        }
    }

    return true;
}

const char *cm_part_filename(cert_manager_part_t part)
{
    switch (part)
    {
        case CERT_MANAGER_CA:          return "ca.pem";
        case CERT_MANAGER_CLIENT_CERT: return "client.crt";
        case CERT_MANAGER_CLIENT_KEY:  return "client.key";
        default:                       return NULL;
    }
}

bool cm_part_from_field(const char *field, cert_manager_part_t *out)
{
    if (field == NULL || out == NULL)
    {
        return false;
    }

    /* the legacy multipart form's field names — preserved so the
     * existing UI form posts keep working */
    if (strcmp(field, "ca") == 0)
    {
        *out = CERT_MANAGER_CA;
    }
    else if (strcmp(field, "client_cert") == 0)
    {
        *out = CERT_MANAGER_CLIENT_CERT;
    }
    else if (strcmp(field, "client_key") == 0)
    {
        *out = CERT_MANAGER_CLIENT_KEY;
    }
    else
    {
        return false;
    }

    return true;
}

bool cm_part_from_type(const char *type, cert_manager_part_t *out)
{
    if (type == NULL || out == NULL)
    {
        return false;
    }

    if (strcmp(type, "ca") == 0)
    {
        *out = CERT_MANAGER_CA;
    }
    else if (strcmp(type, "cert") == 0)
    {
        *out = CERT_MANAGER_CLIENT_CERT;
    }
    else if (strcmp(type, "key") == 0)
    {
        *out = CERT_MANAGER_CLIENT_KEY;
    }
    else
    {
        return false;
    }

    return true;
}

/** memmem-lite: @p needle (string) anywhere in @p hay. */
static bool contains(const char *hay, size_t hay_len, const char *needle)
{
    size_t nlen = strlen(needle);

    if (nlen == 0 || hay_len < nlen)
    {
        return false;
    }

    for (size_t i = 0; i + nlen <= hay_len; i++)
    {
        if (memcmp(hay + i, needle, nlen) == 0)
        {
            return true;
        }
    }

    return false;
}

bool cm_pem_plausible(cert_manager_part_t part, const char *data,
                      size_t len)
{
    if (data == NULL || len < 32 || len > CERT_MANAGER_PEM_MAX)
    {
        return false;
    }

    /* PEM is printable ASCII + newlines; a DER/binary upload is the
     * classic wrong-file mistake */
    for (size_t i = 0; i < len; i++)
    {
        char c = data[i];

        if (c != '\n' && c != '\r' && c != '\t' && (c < 0x20 || c > 0x7E))
        {
            return false;
        }
    }

    if (part == CERT_MANAGER_CLIENT_KEY)
    {
        return contains(data, len, "-----BEGIN PRIVATE KEY-----") ||
               contains(data, len, "-----BEGIN RSA PRIVATE KEY-----") ||
               contains(data, len, "-----BEGIN EC PRIVATE KEY-----");
    }

    return contains(data, len, "-----BEGIN CERTIFICATE-----");
}

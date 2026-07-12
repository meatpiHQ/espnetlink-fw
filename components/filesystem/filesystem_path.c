/**
 * @file filesystem_path.c
 * @brief Pure path validation/routing for the filesystem component.
 *        No IDF/VFS dependencies — compiled as-is by the host unit tests.
 */
#include "filesystem_private.h"

#include <string.h>

typedef struct
{
    const char  *prefix;
    size_t       len;
    fs_backend_t backend;
} prefix_map_t;

static const prefix_map_t PREFIXES[] =
{
    { FS_PREFIX_INTERNAL, sizeof(FS_PREFIX_INTERNAL) - 1, FS_BACKEND_INTERNAL },
    { FS_PREFIX_SD,       sizeof(FS_PREFIX_SD) - 1,       FS_BACKEND_SD       },
};

static bool char_ok(char c)
{
    /* printable ASCII, minus backslash (path-separator confusion) */
    return (c >= 0x20) && (c <= 0x7e) && (c != '\\');
}

static bool segment_ok(const char *seg, size_t len)
{
    if (len == 0)
    {
        return false; /* "//" */
    }

    if ((len == 1 && seg[0] == '.') ||
        (len == 2 && seg[0] == '.' && seg[1] == '.'))
    {
        return false; /* "." / ".." */
    }

    for (size_t i = 0; i < len; i++)
    {
        if (!char_ok(seg[i]))
        {
            return false;
        }
    }

    return true;
}

esp_err_t fs_path_resolve(const char *path, fs_backend_t *out_backend)
{
    if (path == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strnlen(path, FS_PATH_MAX);

    if (len == 0 || len >= FS_PATH_MAX || path[0] != '/')
    {
        return ESP_ERR_INVALID_ARG;
    }

    const prefix_map_t *hit = NULL;

    for (size_t i = 0; i < sizeof(PREFIXES) / sizeof(PREFIXES[0]); i++)
    {
        const prefix_map_t *p = &PREFIXES[i];

        if (strncmp(path, p->prefix, p->len) == 0 &&
            (path[p->len] == '\0' || path[p->len] == '/'))
        {
            hit = p;
            break;
        }
    }

    if (hit == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* validate the remainder segment by segment */
    const char *s = path + hit->len;

    while (*s == '/')
    {
        const char *seg = s + 1;
        const char *end = strchr(seg, '/');
        size_t      seg_len = (end != NULL) ? (size_t)(end - seg)
                                            : strlen(seg);

        if (!segment_ok(seg, seg_len))
        {
            return ESP_ERR_INVALID_ARG;
        }

        s = seg + seg_len;
    }

    if (*s != '\0')
    {
        return ESP_ERR_INVALID_ARG; /* junk directly after the prefix */
    }

    if (path[len - 1] == '/')
    {
        return ESP_ERR_INVALID_ARG; /* trailing slash (covers "/data/") */
    }

    if (out_backend != NULL)
    {
        *out_backend = hit->backend;
    }

    return ESP_OK;
}

esp_err_t fs_path_temp_name(const char *path, char *out, size_t out_len)
{
    if (path == NULL || out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t need = strlen(path) + sizeof(FS_TEMP_SUFFIX); /* incl. NUL */

    if (need > out_len || need > FS_PATH_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    strcpy(out, path);
    strcat(out, FS_TEMP_SUFFIX);
    return ESP_OK;
}

esp_err_t fs_path_parent(const char *path, char *out, size_t out_len)
{
    if (path == NULL || out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const char *last = strrchr(path, '/');

    if (last == NULL || last == path)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t plen = (size_t)(last - path);

    if (plen + 1 > out_len)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(out, path, plen);
    out[plen] = '\0';
    return ESP_OK;
}

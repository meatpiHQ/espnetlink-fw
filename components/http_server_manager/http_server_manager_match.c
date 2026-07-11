/**
 * @file http_server_manager_match.c
 * @brief URI normalization, asset-table resolution, MIME inference.
 *        Pure logic, no httpd/VFS — host-testable on the linux target.
 */
#include <string.h>
#include <strings.h>
#include <stdio.h>

#include "http_server_manager_private.h"

esp_err_t hsm_match_normalize(const char *uri, char *out, size_t out_len)
{
    if (uri == NULL || out == NULL || out_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Reject path traversal before anything touches the filesystem. */
    if (strstr(uri, "..") != NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t i = 0;

    while (uri[i] != '\0' && uri[i] != '?' && uri[i] != '#')
    {
        if (i >= out_len - 1)
        {
            return ESP_ERR_INVALID_ARG;   /* over-long: refuse, don't truncate */
        }

        out[i] = uri[i];
        i++;
    }

    out[i] = '\0';

    /* Root maps to the conventional index page. */
    if (i == 1 && out[0] == '/')
    {
        if (out_len < sizeof("/index.html"))
        {
            return ESP_ERR_INVALID_ARG;
        }

        strcpy(out, "/index.html");
    }

    return (i > 0) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

/* True if @p entry_uri is a prefix pattern (slash-then-asterisk suffix); prefix_len_out receives
   the length of the fixed part including the trailing slash ("/web/" -> 5). */
static bool is_prefix_pattern(const char *entry_uri, size_t *prefix_len_out)
{
    size_t len = strlen(entry_uri);

    if (len >= 2 && entry_uri[len - 1] == '*' && entry_uri[len - 2] == '/')
    {
        *prefix_len_out = len - 1;   /* keep the slash, drop the '*' */
        return true;
    }

    return false;
}

static const http_asset_t *match_in_table(const http_asset_t *table,
                                          const char *uri,
                                          char *fs_path_out, size_t fs_path_len)
{
    for (const http_asset_t *e = table; e->uri != NULL; e++)
    {
        size_t prefix_len = 0;

        if (is_prefix_pattern(e->uri, &prefix_len))
        {
            if (strncmp(uri, e->uri, prefix_len) != 0 ||
                uri[prefix_len] == '\0' || e->fs_path == NULL)
            {
                continue;   /* prefix entries serve files under a directory */
            }

            int n = snprintf(fs_path_out, fs_path_len, "%s/%s",
                             e->fs_path, uri + prefix_len);

            if (n < 0 || (size_t)n >= fs_path_len)
            {
                continue;   /* resolved path would overflow: not a match */
            }

            return e;
        }

        if (strcmp(uri, e->uri) == 0)
        {
            if (e->data_start == NULL && e->fs_path != NULL)
            {
                int n = snprintf(fs_path_out, fs_path_len, "%s", e->fs_path);

                if (n < 0 || (size_t)n >= fs_path_len)
                {
                    continue;
                }
            }

            return e;
        }
    }

    return NULL;
}

const http_asset_t *hsm_match_resolve(const http_asset_t *const *tables,
                                      size_t table_count, const char *uri,
                                      char *fs_path_out, size_t fs_path_len)
{
    if (tables == NULL || uri == NULL || fs_path_out == NULL)
    {
        return NULL;
    }

    for (size_t t = 0; t < table_count; t++)
    {
        const http_asset_t *hit =
            match_in_table(tables[t], uri, fs_path_out, fs_path_len);

        if (hit != NULL)
        {
            return hit;   /* earlier-registered tables win */
        }
    }

    return NULL;
}

typedef struct
{
    const char *ext;
    const char *mime;
} mime_entry_t;

static const mime_entry_t MIME_TABLE[] =
{
    { ".html",  "text/html" },
    { ".htm",   "text/html" },
    { ".js",    "application/javascript" },
    { ".mjs",   "application/javascript" },
    { ".css",   "text/css" },
    { ".json",  "application/json" },
    { ".map",   "application/json" },
    { ".png",   "image/png" },
    { ".jpg",   "image/jpeg" },
    { ".jpeg",  "image/jpeg" },
    { ".gif",   "image/gif" },
    { ".svg",   "image/svg+xml" },
    { ".ico",   "image/x-icon" },
    { ".wasm",  "application/wasm" },
    { ".txt",   "text/plain" },
    { ".pdf",   "application/pdf" },
    { ".woff2", "font/woff2" },
    { ".woff",  "font/woff" },
};

const char *hsm_match_mime_from_path(const char *path)
{
    if (path != NULL)
    {
        const char *dot = strrchr(path, '.');

        if (dot != NULL)
        {
            for (size_t i = 0; i < sizeof(MIME_TABLE) / sizeof(MIME_TABLE[0]); i++)
            {
                if (strcasecmp(dot, MIME_TABLE[i].ext) == 0)
                {
                    return MIME_TABLE[i].mime;
                }
            }
        }
    }

    return "application/octet-stream";
}

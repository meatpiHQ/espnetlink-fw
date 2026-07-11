/**
 * @file http_server_manager_serve.c
 * @brief The GET catch-all: resolves a request against the registered asset
 *        tables and serves embedded blobs or filesystem files, with
 *        fetch-on-miss caching and ETag/304 handling. Target-only.
 */
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "filesystem.h"

#include "http_server_manager_private.h"

static const char *TAG = "http_server_manager";

#define HSM_CHUNK_SIZE 4096
#define HSM_ETAG_LEN   48

/* Weak ETag from identity (size + mtime; embedded assets have no mtime). */
static void make_etag(char *buf, size_t buf_len, size_t size, long mtime)
{
    snprintf(buf, buf_len, "W/\"%lx-%lx\"", (unsigned long)size,
             (unsigned long)mtime);
}

/* True if the request carries If-None-Match equal to @p etag -> reply 304. */
static bool client_has_current(httpd_req_t *req, const char *etag)
{
    char inm[HSM_ETAG_LEN];

    if (httpd_req_get_hdr_value_str(req, "If-None-Match", inm, sizeof(inm)) != ESP_OK)
    {
        return false;
    }

    return strcmp(inm, etag) == 0;
}

static void set_common_headers(httpd_req_t *req, const http_asset_t *e,
                               const char *resolved_path, const char *etag)
{
    const char *mime = (e->content_type != NULL)
                           ? e->content_type
                           : hsm_match_mime_from_path(resolved_path);

    httpd_resp_set_type(req, mime);

    if (e->content_encoding != NULL)
    {
        httpd_resp_set_hdr(req, "Content-Encoding", e->content_encoding);
    }

    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600");
    httpd_resp_set_hdr(req, "ETag", etag);
}

static esp_err_t reply_304(httpd_req_t *req, const char *etag)
{
    httpd_resp_set_status(req, "304 Not Modified");
    httpd_resp_set_hdr(req, "ETag", etag);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t serve_embedded(httpd_req_t *req, const http_asset_t *e,
                                const char *uri)
{
    size_t size = (size_t)(e->data_end - e->data_start);
    char   etag[HSM_ETAG_LEN];

    make_etag(etag, sizeof(etag), size, 0);

    if (client_has_current(req, etag))
    {
        return reply_304(req, etag);
    }

    set_common_headers(req, e, uri, etag);
    return httpd_resp_send(req, (const char *)e->data_start, size);
}

static esp_err_t serve_file(httpd_req_t *req, const http_asset_t *e,
                            const char *path)
{
    /* All file access goes through the filesystem component: it owns the
     * mounts, validates the path, and knows whether the backend (internal
     * or SD) is currently available (ARCHITECTURE §9.4). */
    FILE *f = filesystem_open(path, "rb");

    if (f == NULL)
    {
        /* Miss: fetch-once so the next request is served locally. */
        http_asset_fetch_fn_t fetch = hsm_asset_fetcher();

        if (e->source_url == NULL || fetch == NULL)
        {
            /* DEBUG: per-request path — scanners/bad links must not be able
               to flood the log pipeline at WARN (§10 hot-path rule) */
            ESP_LOGD(TAG, "404 %s (no local file%s)", path,
                     e->source_url != NULL ? ", no fetcher wired" : "");
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
            return ESP_OK;
        }

        ESP_LOGI(TAG, "miss %s -> fetching %s", path, e->source_url);
        esp_err_t err = fetch(e->source_url, path);

        if (err == ESP_OK)
        {
            f = filesystem_open(path, "rb");
        }

        if (f == NULL)
        {
            ESP_LOGE(TAG, "fetch failed for %s (%s)", e->source_url,
                     esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                                "Not cached and fetch failed");
            return ESP_OK;
        }
    }

    struct stat st = { 0 };

    fstat(fileno(f), &st);

    char etag[HSM_ETAG_LEN];
    make_etag(etag, sizeof(etag), (size_t)st.st_size, (long)st.st_mtime);

    if (client_has_current(req, etag))
    {
        fclose(f);
        return reply_304(req, etag);
    }

    /* internal: DMA / cache-off during FS read (SD DMA, LittleFS cache-off). */
    char *chunk = heap_caps_malloc(HSM_CHUNK_SIZE, MALLOC_CAP_INTERNAL);

    if (chunk == NULL)
    {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
        return ESP_OK;
    }

    set_common_headers(req, e, path, etag);

    esp_err_t result = ESP_OK;
    size_t    n;

    while ((n = fread(chunk, 1, HSM_CHUNK_SIZE, f)) > 0)
    {
        if (httpd_resp_send_chunk(req, chunk, n) != ESP_OK)
        {
            /* DEBUG: flaky clients can abort every request (§10 hot path) */
            ESP_LOGD(TAG, "client aborted %s", path);
            httpd_resp_sendstr_chunk(req, NULL);   /* terminate response */
            result = ESP_FAIL;
            break;
        }
    }

    if (result == ESP_OK)
    {
        httpd_resp_send_chunk(req, NULL, 0);   /* end of chunked body */
    }

    free(chunk);
    fclose(f);
    return result;
}

esp_err_t hsm_serve_catchall(httpd_req_t *req)
{
    char uri[HSM_PATH_MAX];
    char fs_path[HSM_PATH_MAX];

    if (hsm_match_normalize(req->uri, uri, sizeof(uri)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad path");
        return ESP_OK;
    }

    size_t table_count = 0;
    const http_asset_t *const *tables = hsm_asset_tables(&table_count);

    const http_asset_t *e =
        hsm_match_resolve(tables, table_count, uri, fs_path, sizeof(fs_path));

    if (e == NULL)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_OK;
    }

    if (e->data_start != NULL)
    {
        return serve_embedded(req, e, uri);
    }

    return serve_file(req, e, fs_path);
}

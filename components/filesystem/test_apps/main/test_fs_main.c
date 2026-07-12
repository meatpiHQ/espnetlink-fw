/**
 * @file test_fs_main.c
 * @brief On-target test app for the filesystem component. Self-contained:
 *        exercises the real `storage` LittleFS partition (from the MAIN
 *        firmware's partition table, Coding Standard rev 2.1 §7) and prints
 *        markers that pytest_filesystem.py asserts in order.
 */
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "filesystem.h"

static const char *TAG = "fs_test";

typedef struct
{
    int  files;
    int  dirs;
    bool saw_file;
    bool saw_dir;
} list_ctx_t;

static esp_err_t list_cb(const char *name, bool is_dir, size_t size,
                         void *ctx)
{
    list_ctx_t *c = ctx;

    (void)size;

    if (is_dir)
    {
        c->dirs++;

        if (strcmp(name, "b") == 0)
        {
            c->saw_dir = true;
        }
    }
    else
    {
        c->files++;

        if (strcmp(name, "hello.txt") == 0)
        {
            c->saw_file = true;
        }
    }

    return ESP_OK;
}

void app_main(void)
{
    char   buf[64];
    size_t n = 0;

    /* ---- mount ---------------------------------------------------------- */
    esp_err_t err = filesystem_init();

    printf("MOUNT ok=%d\n", err == ESP_OK);
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(filesystem_start());

    /* clean slate so re-runs behave like first runs */
    filesystem_delete("/data/t/hello.txt");
    filesystem_delete("/data/t/a/b/deep.txt");

    /* ---- write / read roundtrip ----------------------------------------- */
    ESP_ERROR_CHECK(filesystem_write("/data/t/hello.txt", "hello-fs", 8));
    memset(buf, 0, sizeof(buf));
    ESP_ERROR_CHECK(filesystem_read("/data/t/hello.txt", buf, sizeof(buf), &n));
    printf("RW body=%.*s len=%u\n", (int)n, buf, (unsigned)n);

    /* ---- exists ---------------------------------------------------------- */
    printf("EXISTS file=%d missing=%d\n",
           filesystem_exists("/data/t/hello.txt"),
           filesystem_exists("/data/t/nope.txt"));

    /* ---- atomic overwrite: new content, no temp leftover ----------------- */
    ESP_ERROR_CHECK(filesystem_write("/data/t/hello.txt", "v2-content", 10));
    memset(buf, 0, sizeof(buf));
    ESP_ERROR_CHECK(filesystem_read("/data/t/hello.txt", buf, sizeof(buf), &n));
    printf("ATOMIC body=%.*s leftover=%d\n", (int)n, buf,
           filesystem_exists("/data/t/hello.txt.tmp"));

    /* ---- nested parent auto-create --------------------------------------- */
    ESP_ERROR_CHECK(filesystem_write("/data/t/a/b/deep.txt", "deep", 4));
    memset(buf, 0, sizeof(buf));
    ESP_ERROR_CHECK(filesystem_read("/data/t/a/b/deep.txt", buf, sizeof(buf), &n));
    printf("NESTED body=%.*s\n", (int)n, buf);

    /* ---- size + too-small buffer ----------------------------------------- */
    size_t sz = 0;

    ESP_ERROR_CHECK(filesystem_size("/data/t/hello.txt", &sz));

    char tiny[4];
    esp_err_t small = filesystem_read("/data/t/hello.txt", tiny, sizeof(tiny), &n);

    printf("SIZE n=%u small=%s need=%u\n", (unsigned)sz,
           esp_err_to_name(small), (unsigned)n);

    /* ---- list ------------------------------------------------------------- */
    list_ctx_t lc = { 0 };

    ESP_ERROR_CHECK(filesystem_list("/data/t", list_cb, &lc));
    /* /data/t contains hello.txt + dir a; /data/t/a contains dir b */
    list_ctx_t lc2 = { 0 };

    ESP_ERROR_CHECK(filesystem_list("/data/t/a", list_cb, &lc2));
    printf("LIST saw_file=%d saw_dir=%d\n", lc.saw_file, lc2.saw_dir);

    /* ---- delete ------------------------------------------------------------ */
    ESP_ERROR_CHECK(filesystem_delete("/data/t/hello.txt"));
    printf("DELETE ok=1 exists=%d\n", filesystem_exists("/data/t/hello.txt"));

    /* ---- invalid paths ------------------------------------------------------ */
    const char *bad[] =
    {
        "../x", "/data/../etc", "/nope/x", "/data//x", "/data/a/",
    };
    int rejected = 0;

    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
    {
        if (filesystem_write(bad[i], "x", 1) == ESP_ERR_INVALID_ARG)
        {
            rejected++;
        }
    }

    printf("BADPATH rejected=%d\n", rejected);

    /* ---- /sd reserved but unavailable in v1 --------------------------------- */
    printf("SDPATH state=%d\n",
           filesystem_write("/sd/x", "x", 1) == ESP_ERR_INVALID_STATE);

    /* ---- streaming open ------------------------------------------------------ */
    FILE *f = filesystem_open("/data/t/a/b/deep.txt", "rb");

    if (f != NULL)
    {
        memset(buf, 0, sizeof(buf));
        n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);
        printf("OPEN body=%.*s\n", (int)n, buf);
    }

    /* ---- capacity ------------------------------------------------------------- */
    size_t total = 0;
    size_t used = 0;

    ESP_ERROR_CHECK(filesystem_info("/data", &total, &used));
    printf("INFO total_kib=%u used_ok=%d\n", (unsigned)(total / 1024),
           used > 0);

    /* ---- persistence across remount (stop -> init) ----------------------------- */
    ESP_ERROR_CHECK(filesystem_stop());
    ESP_ERROR_CHECK(filesystem_init());
    ESP_ERROR_CHECK(filesystem_start());
    memset(buf, 0, sizeof(buf));
    ESP_ERROR_CHECK(filesystem_read("/data/t/a/b/deep.txt", buf, sizeof(buf), &n));
    printf("REMOUNT body=%.*s\n", (int)n, buf);

    ESP_LOGI(TAG, "suite finished");
    printf("TEST DONE\n");
}

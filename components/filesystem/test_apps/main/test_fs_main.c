/**
 * @file test_fs_main.c
 * @brief On-target test for the filesystem shim on the MAIN partition
 *        table's `storage` LittleFS partition: mount, POSIX write/read
 *        roundtrip, delete, and content survival across a remount-less
 *        reopen. Markers are consumed by pytest_filesystem.py.
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "filesystem.h"

static const char *PATH = "/storage/fs_test_marker.txt";
static const char *PAYLOAD = "espnetlink-fs-roundtrip";

void app_main(void)
{
    if (filesystem_init() != ESP_OK)
    {
        printf("FS MOUNT FAIL\n");
        return;
    }

    printf("FS MOUNT OK\n");

    FILE *f = fopen(PATH, "w");

    if (f == NULL)
    {
        printf("FS WRITE FAIL open\n");
        return;
    }

    fputs(PAYLOAD, f);
    fclose(f);
    printf("FS WRITE OK\n");

    char buf[64] = "";

    f = fopen(PATH, "r");

    if (f == NULL || fgets(buf, sizeof(buf), f) == NULL)
    {
        printf("FS READ FAIL\n");
        return;
    }

    fclose(f);
    printf("FS READ OK content=%s\n", buf);

    if (strcmp(buf, PAYLOAD) != 0)
    {
        printf("FS CONTENT MISMATCH\n");
        return;
    }

    if (unlink(PATH) != 0)
    {
        printf("FS DELETE FAIL\n");
        return;
    }

    printf("FS DELETE OK\n");

    if (fopen(PATH, "r") != NULL)
    {
        printf("FS DELETE NOT EFFECTIVE\n");
        return;
    }

    printf("TEST DONE\n");
}

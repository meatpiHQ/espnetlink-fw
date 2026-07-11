#pragma once

#include <stdio.h>

#include "esp_err.h"

#define FS_BASE_PATH "/storage"

esp_err_t filesystem_init(void);

/**
 * Open a file through the filesystem component (the WiCAN shared-core
 * surface consumed by http_server_manager). The component owns the mounts;
 * callers never touch VFS registration themselves. NULL when the path can't
 * be opened.
 */
FILE *filesystem_open(const char *path, const char *mode);

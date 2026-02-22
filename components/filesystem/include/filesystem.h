#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include "esp_err.h"

#define FS_BASE_PATH "/storage"

esp_err_t filesystem_init(void);

#endif // FILESYSTEM_H

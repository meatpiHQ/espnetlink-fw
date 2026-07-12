/**
 * @file filesystem.c
 * @brief Lifecycle and file operations for the WiCAN filesystem component.
 *
 * Owns the internal-flash LittleFS mount (/data). The /sd backend is
 * reserved for external_storage and reports ESP_ERR_INVALID_STATE until
 * that component exists and wires it up. Settings live in
 * filesystem_settings.c (standard §4.1).
 */
#include "filesystem.h"

#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_attr.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include "log_manager.h"

#include "filesystem_private.h"

static const char *TAG = "filesystem";

typedef struct
{
    bool mounted;
} backend_state_t;

typedef struct
{
    bool            inited;
    bool            started;
    backend_state_t backend[FS_BACKEND_COUNT];
} fs_state_t;

static fs_state_t s_fs EXT_RAM_BSS_ATTR; /* tiny; PSRAM by default (§2) */

static StaticSemaphore_t s_lock_buf; /* internal: FreeRTOS object */
static SemaphoreHandle_t s_lock;

/* Bounce buffer so caller data (possibly PSRAM) is never handed to the
 * flash path directly. */
static uint8_t s_scratch[2048]; /* internal: cache-off during FS write */

#define FS_LOCK()   xSemaphoreTakeRecursive(s_lock, portMAX_DELAY)
#define FS_UNLOCK() xSemaphoreGiveRecursive(s_lock)

/* ---- helpers ----------------------------------------------------------- */

static esp_err_t errno_to_esp(int e)
{
    switch (e)
    {
        case ENOENT:  return ESP_ERR_NOT_FOUND;
        case ENOMEM:  return ESP_ERR_NO_MEM;
        case ENOSPC:  return ESP_ERR_NO_MEM;
        case EINVAL:  return ESP_ERR_INVALID_ARG;
        default:      return ESP_FAIL;
    }
}

/** Validate the path and confirm its backend is usable right now. */
static esp_err_t check_path(const char *path, fs_backend_t *out_backend)
{
    fs_backend_t backend;
    esp_err_t    err = fs_path_resolve(path, &backend);

    if (err != ESP_OK)
    {
        return err;
    }

    if (!s_fs.inited || !s_fs.backend[backend].mounted)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_backend != NULL)
    {
        *out_backend = backend;
    }

    return ESP_OK;
}

static esp_err_t mkdirs_locked(const char *dir_path)
{
    char partial[FS_PATH_MAX];
    size_t len = strlen(dir_path);

    if (len >= sizeof(partial))
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* walk segment by segment, creating as we go; the prefix root ("/data")
     * always exists as the mount point */
    for (size_t i = 1; i <= len; i++)
    {
        if (dir_path[i] == '/' || dir_path[i] == '\0')
        {
            memcpy(partial, dir_path, i);
            partial[i] = '\0';

            if (strcmp(partial, FS_PREFIX_INTERNAL) == 0 ||
                strcmp(partial, FS_PREFIX_SD) == 0)
            {
                continue;
            }

            if (mkdir(partial, 0775) != 0 && errno != EEXIST)
            {
                ESP_LOGE(TAG, "mkdir '%s' failed: errno %d", partial, errno);
                return errno_to_esp(errno);
            }
        }
    }

    return ESP_OK;
}

/* ---- lifecycle ---------------------------------------------------------- */

esp_err_t filesystem_init(void)
{
    if (s_fs.inited)
    {
        return ESP_OK;
    }

    static const log_descriptor_t LOG_DESC = { "filesystem", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC); /* per-TAG level control (§9.2) */

    esp_err_t stream_err = fs_stream_service_init();

    if (stream_err != ESP_OK)
    {
        ESP_LOGE(TAG, "async writer service failed: %s (streamed writes "
                 "unavailable)", esp_err_to_name(stream_err));
    }

    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_buf);
    }

    esp_vfs_littlefs_conf_t conf =
    {
        .base_path              = FS_PREFIX_INTERNAL,
        .partition_label        = CONFIG_FILESYSTEM_PARTITION_LABEL,
        .format_if_mount_failed = true,
        .dont_mount             = false,
    };

    esp_err_t err = esp_vfs_littlefs_register(&conf);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "mount '%s' at %s failed: %s",
                 CONFIG_FILESYSTEM_PARTITION_LABEL, FS_PREFIX_INTERNAL,
                 esp_err_to_name(err));
        return err;
    }

    s_fs.backend[FS_BACKEND_INTERNAL].mounted = true;
    s_fs.inited = true;

    size_t total = 0;
    size_t used = 0;

    if (esp_littlefs_info(CONFIG_FILESYSTEM_PARTITION_LABEL,
                          &total, &used) == ESP_OK)
    {
        ESP_LOGI(TAG, "mounted %s (%u KiB used / %u KiB)",
                 FS_PREFIX_INTERNAL,
                 (unsigned)(used / 1024), (unsigned)(total / 1024));
    }

    return ESP_OK;
}

esp_err_t filesystem_start(void)
{
    if (!s_fs.inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    s_fs.started = true;
    return ESP_OK;
}

esp_err_t filesystem_stop(void)
{
    if (!s_fs.inited)
    {
        return ESP_OK;
    }

    FS_LOCK();

    esp_err_t err = esp_vfs_littlefs_unregister(
        CONFIG_FILESYSTEM_PARTITION_LABEL);

    s_fs.backend[FS_BACKEND_INTERNAL].mounted = false;
    s_fs.inited = false;
    s_fs.started = false;

    FS_UNLOCK();
    return err;
}

esp_err_t filesystem_sd_set_mounted(bool mounted)
{
    if (!s_fs.inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    FS_LOCK();
    s_fs.backend[FS_BACKEND_SD].mounted = mounted;
    FS_UNLOCK();
    ESP_LOGI(TAG, "/sd backend %s", mounted ? "enabled" : "disabled");
    return ESP_OK;
}

/* ---- operations ---------------------------------------------------------- */

esp_err_t filesystem_write(const char *path, const void *data, size_t len)
{
    esp_err_t err = check_path(path, NULL);

    if (err != ESP_OK)
    {
        return err;
    }

    if (data == NULL && len > 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char tmp[FS_PATH_MAX];
    char parent[FS_PATH_MAX];

    err = fs_path_temp_name(path, tmp, sizeof(tmp));

    if (err != ESP_OK)
    {
        return err;
    }

    FS_LOCK();

    if (fs_path_parent(path, parent, sizeof(parent)) == ESP_OK)
    {
        err = mkdirs_locked(parent);

        if (err != ESP_OK)
        {
            FS_UNLOCK();
            return err;
        }
    }

    FILE *f = fopen(tmp, "wb");

    if (f == NULL)
    {
        ESP_LOGE(TAG, "open '%s' for write failed: errno %d", tmp, errno);
        FS_UNLOCK();
        return errno_to_esp(errno);
    }

    const uint8_t *src = data;
    size_t         left = len;

    while (left > 0)
    {
        size_t chunk = (left > sizeof(s_scratch)) ? sizeof(s_scratch) : left;

        memcpy(s_scratch, src, chunk); /* PSRAM -> internal, cache on */

        if (fwrite(s_scratch, 1, chunk, f) != chunk)
        {
            ESP_LOGE(TAG, "write '%s' failed: errno %d", path, errno);
            fclose(f);
            unlink(tmp);
            FS_UNLOCK();
            return ESP_FAIL;
        }

        src += chunk;
        left -= chunk;
    }

    fflush(f);
    fsync(fileno(f)); /* force to media before the rename (REVIEW M-4) */

    if (fclose(f) != 0)
    {
        unlink(tmp);
        FS_UNLOCK();
        return ESP_FAIL;
    }

    if (rename(tmp, path) != 0)
    {
        ESP_LOGE(TAG, "rename '%s' -> '%s' failed: errno %d",
                 tmp, path, errno);
        unlink(tmp);
        FS_UNLOCK();
        return errno_to_esp(errno);
    }

    FS_UNLOCK();
    return ESP_OK;
}

/* ---- internals shared with filesystem_stream.c ------------------------------- */

esp_err_t fs_check_path(const char *path, fs_backend_t *out_backend)
{
    return check_path(path, out_backend);
}

esp_err_t fs_mkdirs_in_lock(const char *dir_path)
{
    return mkdirs_locked(dir_path);
}

void fs_lock(void)
{
    FS_LOCK();
}

void fs_unlock(void)
{
    FS_UNLOCK();
}

uint8_t *fs_scratch(size_t *size_out)
{
    *size_out = sizeof(s_scratch);
    return s_scratch;
}

esp_err_t fs_errno_to_esp(int err)
{
    return errno_to_esp(err);
}

esp_err_t filesystem_read(const char *path, void *buf, size_t buf_len,
                          size_t *out_len)
{
    esp_err_t err = check_path(path, NULL);

    if (err != ESP_OK)
    {
        return err;
    }

    if (buf == NULL && buf_len > 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    FS_LOCK();

    FILE *f = fopen(path, "rb");

    if (f == NULL)
    {
        FS_UNLOCK();
        return errno_to_esp(errno);
    }

    struct stat st;

    if (fstat(fileno(f), &st) != 0)
    {
        fclose(f);
        FS_UNLOCK();
        return ESP_FAIL;
    }

    size_t size = (size_t)st.st_size;

    if (out_len != NULL)
    {
        *out_len = size;
    }

    if (size > buf_len)
    {
        fclose(f);
        FS_UNLOCK();
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *dst = buf;
    size_t   left = size;

    while (left > 0)
    {
        size_t chunk = (left > sizeof(s_scratch)) ? sizeof(s_scratch) : left;

        if (fread(s_scratch, 1, chunk, f) != chunk)
        {
            fclose(f);
            FS_UNLOCK();
            return ESP_FAIL;
        }

        memcpy(dst, s_scratch, chunk); /* internal -> caller (maybe PSRAM) */
        dst += chunk;
        left -= chunk;
    }

    fclose(f);
    FS_UNLOCK();
    return ESP_OK;
}

esp_err_t filesystem_size(const char *path, size_t *out_size)
{
    esp_err_t err = check_path(path, NULL);

    if (err != ESP_OK)
    {
        return err;
    }

    if (out_size == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    FS_LOCK();

    struct stat st;
    int rc = stat(path, &st);

    FS_UNLOCK();

    if (rc != 0)
    {
        return errno_to_esp(errno);
    }

    *out_size = (size_t)st.st_size;
    return ESP_OK;
}

bool filesystem_exists(const char *path)
{
    if (check_path(path, NULL) != ESP_OK)
    {
        return false;
    }

    FS_LOCK();

    struct stat st;
    int rc = stat(path, &st);

    FS_UNLOCK();
    return rc == 0;
}

esp_err_t filesystem_delete(const char *path)
{
    esp_err_t err = check_path(path, NULL);

    if (err != ESP_OK)
    {
        return err;
    }

    FS_LOCK();

    struct stat st;

    if (stat(path, &st) != 0)
    {
        FS_UNLOCK();
        return errno_to_esp(errno);
    }

    int rc = S_ISDIR(st.st_mode) ? rmdir(path) : unlink(path);

    FS_UNLOCK();
    return (rc == 0) ? ESP_OK : errno_to_esp(errno);
}

esp_err_t filesystem_mkdirs(const char *dir_path)
{
    esp_err_t err = check_path(dir_path, NULL);

    if (err != ESP_OK)
    {
        return err;
    }

    FS_LOCK();
    err = mkdirs_locked(dir_path);
    FS_UNLOCK();
    return err;
}

esp_err_t filesystem_list(const char *dir_path, filesystem_list_cb_t cb,
                          void *ctx)
{
    esp_err_t err = check_path(dir_path, NULL);

    if (err != ESP_OK)
    {
        return err;
    }

    if (cb == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    FS_LOCK();

    DIR *dir = opendir(dir_path);

    if (dir == NULL)
    {
        FS_UNLOCK();
        return errno_to_esp(errno);
    }

    struct dirent *ent;

    err = ESP_OK;

    while ((ent = readdir(dir)) != NULL)
    {
        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
        {
            continue;
        }

        char full[FS_PATH_MAX];

        if (snprintf(full, sizeof(full), "%s/%s", dir_path, ent->d_name)
            >= (int)sizeof(full))
        {
            continue; /* name doesn't fit our namespace; skip, don't fail */
        }

        struct stat st;
        bool   is_dir = false;
        size_t size = 0;

        if (stat(full, &st) == 0)
        {
            is_dir = S_ISDIR(st.st_mode);
            size = is_dir ? 0 : (size_t)st.st_size;
        }

        err = cb(ent->d_name, is_dir, size, ctx);

        if (err != ESP_OK)
        {
            break;
        }
    }

    closedir(dir);
    FS_UNLOCK();
    return err;
}

FILE *filesystem_open(const char *path, const char *mode)
{
    if (mode == NULL || check_path(path, NULL) != ESP_OK)
    {
        return NULL;
    }

    return fopen(path, mode);
}

esp_err_t filesystem_info(const char *prefix, size_t *out_total,
                          size_t *out_used)
{
    fs_backend_t backend;
    esp_err_t    err = check_path(prefix, &backend);

    if (err != ESP_OK)
    {
        return err;
    }

    size_t total = 0;
    size_t used = 0;

    if (backend == FS_BACKEND_INTERNAL)
    {
        err = esp_littlefs_info(CONFIG_FILESYSTEM_PARTITION_LABEL, &total,
                                &used);
    }
    else
    {
        /* ESPNetLink deviation: no SD slot / external_storage on this
         * product, so WiCAN's FATFS free-cluster query (esp_vfs_fat_info
         * and the fatfs dependency) is not carried. /sd stays reserved
         * and unmounted; this branch is unreachable until something
         * calls filesystem_sd_set_mounted(true). */
        err = ESP_ERR_INVALID_STATE;
    }

    if (err != ESP_OK)
    {
        return err;
    }

    if (out_total != NULL)
    {
        *out_total = total;
    }

    if (out_used != NULL)
    {
        *out_used = used;
    }

    return ESP_OK;
}

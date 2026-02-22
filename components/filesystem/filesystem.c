#include "filesystem.h"
#include "esp_littlefs.h"
#include "esp_log.h"

static const char *TAG = "filesystem";

esp_err_t filesystem_init(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = FS_BASE_PATH,
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .grow_on_mount = true,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LittleFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0, used = 0;
    esp_littlefs_info("storage", &total, &used);
    ESP_LOGI(TAG, "Mounted at %s — %u KB total, %u KB used",
             FS_BASE_PATH, (unsigned)(total / 1024), (unsigned)(used / 1024));

    return ESP_OK;
}

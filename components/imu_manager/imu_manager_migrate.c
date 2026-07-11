/**
 * @file imu_manager_migrate.c
 * @brief PURE settings migration — cJSON only, host-testable.
 */
#include "cJSON.h"

#include "imu_manager_private.h"

esp_err_t imu_settings_migrate(uint32_t from_version, cJSON *settings)
{
    if (settings == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* v1 was WoM-only (wom_threshold), v2 SMD-only (smd_sensitivity),
     * v3 runs both. Every key either version carried is valid again in
     * v3, so both migrations are pass-throughs — validation fills the
     * missing keys (wom/smd enables etc.) from the schema defaults. */
    return (from_version >= 1) ? ESP_OK : ESP_ERR_INVALID_VERSION;
}

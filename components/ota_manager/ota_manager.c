/**
 * @file ota_manager.c
 * @brief esp_ota glue around the pure session: partition selection, flash
 *        I/O, boot-partition switch, rollback confirmation, and the mutex
 *        that serializes transports.
 */
#include "ota_manager.h"

#include <string.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "log_manager.h"

#include "ota_manager_private.h"

static const char *TAG = "ota_manager";

static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf; /* internal: FreeRTOS object */

static ota_session_t s_session;
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_target;
static bool s_inited;
static ota_manager_event_cb_t s_event_cb;

/** Fire the state-change callback (outside s_lock — the callback may do
 *  I/O, e.g. main's LED-indication glue). */
static void notify(ota_manager_state_t before, ota_manager_state_t after)
{
    if (s_event_cb != NULL && before != after)
    {
        s_event_cb(after);
    }
}

/* ---- the injected flash backend (esp_ota) ------------------------------------ */

static esp_err_t io_begin(void *io, size_t total_size)
{
    (void)io;
    s_target = esp_ota_get_next_update_partition(NULL);

    if (s_target == NULL)
    {
        ESP_LOGE(TAG, "no OTA partition available");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "update session -> %s (%s size)", s_target->label,
             (total_size > 0) ? "announced" : "unknown");
    /* OTA_SIZE_UNKNOWN erases the whole partition up-front; with an
       announced size esp_ota erases incrementally */
    return esp_ota_begin(s_target,
                         (total_size > 0) ? total_size : OTA_SIZE_UNKNOWN,
                         &s_handle);
}

static esp_err_t io_write(void *io, const void *data, size_t len)
{
    (void)io;
    return esp_ota_write(s_handle, data, len);
}

static esp_err_t io_end(void *io)
{
    (void)io;

    esp_err_t err = esp_ota_end(s_handle); /* validates magic + digest */

    if (err == ESP_OK)
    {
        err = esp_ota_set_boot_partition(s_target);
    }

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "image valid; next boot from %s", s_target->label);
    }

    return err;
}

static void io_abort(void *io)
{
    (void)io;
    esp_ota_abort(s_handle);
    ESP_LOGW(TAG, "update session aborted");
}

static const ota_session_ops_t IO_OPS =
{
    .begin = io_begin,
    .write = io_write,
    .end = io_end,
    .abort = io_abort,
};

/* ---- lifecycle ----------------------------------------------------------------- */

esp_err_t ota_manager_init(void)
{
    if (s_inited)
    {
        return ESP_OK;
    }

    static const log_descriptor_t LOG_DESC = { "ota_manager", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC);
    s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    ota_session_init(&s_session, &IO_OPS, NULL);
    s_inited = true;
    return ESP_OK;
}

esp_err_t ota_manager_start(void)
{
    if (!s_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    /* we booted and reached start(): confirm this image so the bootloader
       stops considering a rollback */
    esp_ota_img_states_t img_state;
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (esp_ota_get_state_partition(running, &img_state) == ESP_OK &&
        img_state == ESP_OTA_IMG_PENDING_VERIFY)
    {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "running image confirmed (rollback cancelled)");
    }
#endif

    ESP_LOGI(TAG, "ready (running from %s)",
             esp_ota_get_running_partition()->label);
    return ESP_OK;
}

esp_err_t ota_manager_stop(void)
{
    return ota_manager_abort();
}

/* ---- session API (mutex-serialized; transports call these) --------------------- */

esp_err_t ota_manager_begin(size_t total_size)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    ota_manager_state_t before = s_session.state;
    esp_err_t err = ota_session_begin(&s_session, total_size);
    ota_manager_state_t after = s_session.state;

    xSemaphoreGive(s_lock);
    notify(before, after);
    return err;
}

esp_err_t ota_manager_write(const uint8_t *data, size_t len)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    ota_manager_state_t before = s_session.state;
    esp_err_t err = ota_session_write(&s_session, data, len);
    ota_manager_state_t after = s_session.state;

    xSemaphoreGive(s_lock);
    notify(before, after);
    return err;
}

esp_err_t ota_manager_end(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    ota_manager_state_t before = s_session.state;
    esp_err_t err = ota_session_end(&s_session);
    ota_manager_state_t after = s_session.state;

    xSemaphoreGive(s_lock);
    notify(before, after);
    return err;
}

esp_err_t ota_manager_abort(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    ota_manager_state_t before = s_session.state;
    esp_err_t err = ota_session_abort(&s_session);
    ota_manager_state_t after = s_session.state;

    xSemaphoreGive(s_lock);
    notify(before, after);
    return err;
}

esp_err_t ota_manager_set_event_cb(ota_manager_event_cb_t cb)
{
    s_event_cb = cb; /* pre-start composition glue; one consumer */
    return ESP_OK;
}

esp_err_t ota_manager_status(ota_manager_status_t *out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->state = s_session.state;
    out->received = s_session.received;
    out->total = s_session.total;
    strlcpy(out->error, s_session.error, sizeof(out->error));
    strlcpy(out->target, (s_target != NULL) ? s_target->label : "",
            sizeof(out->target));
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

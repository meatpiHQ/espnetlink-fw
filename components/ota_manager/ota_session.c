/**
 * @file ota_session.c
 * @brief The PURE update-session state machine — no esp_ota, no locks
 *        (the glue serializes); flash operations are injected, so every
 *        transition host-tests without hardware.
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ota_manager_private.h"

static void fail(ota_session_t *s, const char *why, bool abort_backend)
{
    if (abort_backend && s->ops->abort != NULL)
    {
        s->ops->abort(s->io);
    }

    snprintf(s->error, sizeof(s->error), "%s", why);
    s->state = OTA_MANAGER_FAILED;
}

void ota_session_init(ota_session_t *s, const ota_session_ops_t *ops,
                      void *io)
{
    memset(s, 0, sizeof(*s));
    s->ops = ops;
    s->io = io;
    s->state = OTA_MANAGER_IDLE;
}

esp_err_t ota_session_begin(ota_session_t *s, size_t total_size)
{
    if (s->state == OTA_MANAGER_RECEIVING)
    {
        return ESP_ERR_INVALID_STATE; /* one session at a time */
    }

    /* IDLE, FAILED (retry) and READY (re-upload before reboot) may begin */
    s->received = 0;
    s->total = (uint32_t)total_size;
    s->error[0] = '\0';

    esp_err_t err = s->ops->begin(s->io, total_size);

    if (err != ESP_OK)
    {
        fail(s, "flash begin failed", false);
        return err;
    }

    s->state = OTA_MANAGER_RECEIVING;
    return ESP_OK;
}

esp_err_t ota_session_write(ota_session_t *s, const void *data, size_t len)
{
    if (s->state != OTA_MANAGER_RECEIVING)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0)
    {
        fail(s, "empty write", true);
        return ESP_ERR_INVALID_ARG;
    }

    if (s->total != 0 && s->received + len > s->total)
    {
        fail(s, "image larger than announced size", true);
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = s->ops->write(s->io, data, len);

    if (err != ESP_OK)
    {
        fail(s, "flash write failed", true);
        return err;
    }

    s->received += (uint32_t)len;
    return ESP_OK;
}

esp_err_t ota_session_end(ota_session_t *s)
{
    if (s->state != OTA_MANAGER_RECEIVING)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s->received == 0)
    {
        fail(s, "empty image", true);
        return ESP_ERR_INVALID_SIZE;
    }

    if (s->total != 0 && s->received < s->total)
    {
        fail(s, "short image (connection dropped?)", true);
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = s->ops->end(s->io);

    if (err != ESP_OK)
    {
        /* esp_ota_end already released the handle on failure */
        fail(s, "image validation failed", false);
        return err;
    }

    s->state = OTA_MANAGER_READY;
    return ESP_OK;
}

esp_err_t ota_session_abort(ota_session_t *s)
{
    if (s->state == OTA_MANAGER_RECEIVING && s->ops->abort != NULL)
    {
        s->ops->abort(s->io);
    }

    s->state = OTA_MANAGER_IDLE;
    s->received = 0;
    s->total = 0;
    s->error[0] = '\0';
    return ESP_OK;
}

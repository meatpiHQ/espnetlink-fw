/**
 * @file ota_manager.h
 * @brief WiCAN firmware update manager (core service).
 *
 * Owns the device's OWN firmware update: the OTA session over the
 * `ota_0`/`ota_1` app partitions (esp_ota). TRANSPORT-AGNOSTIC by design —
 * the manager exposes one byte-stream session (`begin` → `write`× →
 * `end`/`abort`) and transports feed it:
 *
 *   - today:  HTTP multipart upload from the web UI (`api_http` glue,
 *             `POST /api/ota/upload` — the HTML form path)
 *   - later:  raw HTTP body, TCP, MQTT, … — same four calls, no changes
 *             here (ownership inversion, Architecture §2)
 *
 * One session at a time. `end()` validates the image (esp_ota magic/digest)
 * and sets the boot partition; the TRANSPORT then reboots via
 * `restart_tracker_restart(OTA_APPLY, <source>)` — never from here, and
 * never a raw esp_restart(). A failed/aborted session leaves the running
 * firmware untouched (the inactive partition is scratch space).
 *
 * Naming note: "OTA" = this device's application image. The OBD chip's own
 * firmware update is a different thing (`obd_chip_firmware_update`).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    OTA_MANAGER_IDLE = 0,       /* no session                               */
    OTA_MANAGER_RECEIVING,      /* begin() done, bytes flowing              */
    OTA_MANAGER_READY,          /* end() ok: boot partition set, reboot due */
    OTA_MANAGER_FAILED,         /* session failed; error says why           */
} ota_manager_state_t;

typedef struct
{
    ota_manager_state_t state;
    uint32_t received;          /* bytes written this session               */
    uint32_t total;             /* announced size; 0 = unknown (multipart)  */
    char     error[64];         /* last failure reason ("" when none)       */
    char     target[8];         /* partition being written ("ota_1", …)     */
} ota_manager_status_t;

/** Register the log descriptor. No flash access. */
esp_err_t ota_manager_init(void);

/** Confirm the running image if rollback is enabled; ready for sessions. */
esp_err_t ota_manager_start(void);
esp_err_t ota_manager_stop(void);

/**
 * Open the ONE update session against the next OTA partition.
 * @param total_size announced image size, or 0 when unknown (multipart).
 * ESP_ERR_INVALID_STATE while another session is RECEIVING.
 */
esp_err_t ota_manager_begin(size_t total_size);

/** Stream image bytes (any chunking). Errors latch FAILED + abort flash. */
esp_err_t ota_manager_write(const uint8_t *data, size_t len);

/** Finish: validate the image, set the boot partition → READY. The caller
 *  (transport) reboots via restart_tracker_restart(OTA_APPLY, source). */
esp_err_t ota_manager_end(void);

/** Drop the session; the running firmware is untouched. */
esp_err_t ota_manager_abort(void);

/** Snapshot for progress/diagnostics (GET /api/ota/status). */
esp_err_t ota_manager_status(ota_manager_status_t *out);

/** State-change notification (RECEIVING at begin, READY/FAILED/IDLE at
 *  end/abort). Runs in the TRANSPORT's context, outside the session lock —
 *  keep it short. One consumer: the composition root (e.g. wiring the
 *  "update in progress" LED indication). Register before start(). */
typedef void (*ota_manager_event_cb_t)(ota_manager_state_t state);
esp_err_t ota_manager_set_event_cb(ota_manager_event_cb_t cb);

#ifdef __cplusplus
}
#endif

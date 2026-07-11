// `ota` console command: stream a firmware image over the CDC console into
// the inactive OTA slot.
//
// Wire protocol (every response is a console line; normal OK/ERROR + prompt
// trailer applies after completion):
//   > ota <size_bytes> [sha256_hex]
//   < OTA READY                    (console input is now raw binary)
//   ... host streams exactly <size_bytes> raw image bytes ...
//   < OTA PROG <received>          (every 128 KiB)
//   < OTA DONE <bytes> sha=<hex> -> <partition label>
//
// The handler runs inside the console task (internal-RAM stack: flash writes
// are legal there). RX bytes arrive in USB driver context and are parked in a
// stream buffer; when it fills, the OUT endpoint stays un-armed so the host
// throttles via CDC flow control and nothing is ever dropped.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmdline_manager.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_ota_ops.h"

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "mbedtls/sha256.h"

#include "usb_cli_console_internal.h"

#define OTA_STREAM_BUF_SIZE  (16 * 1024)
#define OTA_WRITE_CHUNK      4096
#define OTA_IDLE_TIMEOUT_MS  15000
#define OTA_POLL_MS          250
#define OTA_PROGRESS_STEP    (128 * 1024)

// Pause RX (host NAK) unless at least two full endpoint reads still fit.
#define OTA_RESUME_HEADROOM  (2 * USB_CLI_RX_BUF_SIZE)

// internal RAM: consumed on the flash-writing console task; PSRAM is not
// enabled in this firmware yet.
static StaticStreamBuffer_t s_sb_struct;
static uint8_t s_sb_storage[OTA_STREAM_BUF_SIZE + 1];
static StreamBufferHandle_t s_sb;

static uint8_t s_chunk[OTA_WRITE_CHUNK]; // static: keep console task stack small

// Runs in USB driver context (possibly ISR). Copy out and never block.
static bool usb_cli_ota_raw_rx(const uint8_t *data, size_t len)
{
    size_t sent;

    if (xPortInIsrContext())
    {
        BaseType_t hp_woken = pdFALSE;

        sent = xStreamBufferSendFromISR(s_sb, data, len, &hp_woken);
        if (hp_woken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        sent = xStreamBufferSend(s_sb, data, len, 0);
    }

    // The pause threshold below guarantees a full endpoint read always fits,
    // so a partial send indicates a logic error, not expected overflow.
    (void)sent;

    return xStreamBufferSpacesAvailable(s_sb) >= OTA_RESUME_HEADROOM;
}

static int usb_cli_ota_parse_sha(const char *hex, uint8_t out[32])
{
    if (strlen(hex) != 64)
    {
        return -1;
    }

    for (int i = 0; i < 32; i++)
    {
        char byte_hex[3] = { hex[2 * i], hex[2 * i + 1], '\0' };
        char *end = NULL;
        long v = strtol(byte_hex, &end, 16);

        if (end == NULL || *end != '\0')
        {
            return -1;
        }
        out[i] = (uint8_t)v;
    }

    return 0;
}

static void usb_cli_ota_hex(const uint8_t *in, size_t len, char *out)
{
    static const char digits[] = "0123456789abcdef";

    for (size_t i = 0; i < len; i++)
    {
        out[2 * i] = digits[in[i] >> 4];
        out[2 * i + 1] = digits[in[i] & 0x0F];
    }
    out[2 * len] = '\0';
}

static int usb_cli_cmd_ota(int argc, char **argv)
{
    bool have_expected_sha = false;
    uint8_t expected_sha[32];

    if (argc < 2 || argc > 3)
    {
        printf("Usage: ota <size_bytes> [sha256_hex]\n");
        return 1;
    }

    long size = strtol(argv[1], NULL, 0);
    if (size <= 0)
    {
        printf("OTA ERR bad size '%s'\n", argv[1]);
        return 1;
    }

    if (argc == 3)
    {
        if (usb_cli_ota_parse_sha(argv[2], expected_sha) != 0)
        {
            printf("OTA ERR bad sha256 (need 64 hex chars)\n");
            return 1;
        }
        have_expected_sha = true;
    }

    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (next == NULL)
    {
        printf("OTA ERR no update partition (partition table has no free OTA slot)\n");
        return 1;
    }

    if ((uint32_t)size > next->size)
    {
        printf("OTA ERR image (%ld) larger than partition %s (%lu)\n",
               size, next->label, (unsigned long)next->size);
        return 1;
    }

    if (s_sb == NULL)
    {
        s_sb = xStreamBufferCreateStatic(sizeof(s_sb_storage) - 1, 1,
                                         s_sb_storage, &s_sb_struct);
        if (s_sb == NULL)
        {
            printf("OTA ERR stream buffer init\n");
            return 1;
        }
    }
    (void)xStreamBufferReset(s_sb);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(next, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK)
    {
        printf("OTA ERR begin: %s\n", esp_err_to_name(err));
        return 1;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);

    usb_cli_console_set_raw_rx(usb_cli_ota_raw_rx);
    printf("OTA READY\n");

    size_t received = 0;
    size_t next_progress = OTA_PROGRESS_STEP;
    uint32_t idle_ms = 0;
    bool failed = false;

    while (received < (size_t)size)
    {
        size_t want = (size_t)size - received;

        if (want > sizeof(s_chunk))
        {
            want = sizeof(s_chunk);
        }

        size_t got = xStreamBufferReceive(s_sb, s_chunk, want,
                                          pdMS_TO_TICKS(OTA_POLL_MS));

        // Reopen the RX pipe if the sink paused it, and retry failed re-arms.
        usb_cli_console_raw_rx_resume();

        if (got == 0)
        {
            idle_ms += OTA_POLL_MS;
            if (idle_ms >= OTA_IDLE_TIMEOUT_MS)
            {
                printf("OTA ERR timeout at %u/%ld bytes\n",
                       (unsigned)received, size);
                failed = true;
                break;
            }
            continue;
        }
        idle_ms = 0;

        mbedtls_sha256_update(&sha, s_chunk, got);

        err = esp_ota_write(handle, s_chunk, got);
        if (err != ESP_OK)
        {
            printf("OTA ERR write at %u: %s\n", (unsigned)received,
                   esp_err_to_name(err));
            failed = true;
            break;
        }

        received += got;

        /* Breathe between flash bursts: erase/write windows run with the
         * cache off and the (non-IRAM) USB interrupt masked — sustained
         * back-to-back bursts can wedge the USB core under RNDIS traffic. */
        if ((received % (32 * 1024)) < OTA_WRITE_CHUNK)
        {
            vTaskDelay(1);
        }

        if (received >= next_progress)
        {
            printf("OTA PROG %u\n", (unsigned)received);
            next_progress += OTA_PROGRESS_STEP;
        }
    }

    usb_cli_console_set_raw_rx(NULL);

    uint8_t actual_sha[32];
    mbedtls_sha256_finish(&sha, actual_sha);
    mbedtls_sha256_free(&sha);

    if (failed)
    {
        esp_ota_abort(handle);
        return 1;
    }

    if (have_expected_sha && memcmp(actual_sha, expected_sha, 32) != 0)
    {
        char hex[65];

        usb_cli_ota_hex(actual_sha, sizeof(actual_sha), hex);
        printf("OTA ERR sha mismatch (got %s)\n", hex);
        esp_ota_abort(handle);
        return 1;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK)
    {
        printf("OTA ERR end (image invalid): %s\n", esp_err_to_name(err));
        return 1;
    }

    err = esp_ota_set_boot_partition(next);
    if (err != ESP_OK)
    {
        printf("OTA ERR set boot partition: %s\n", esp_err_to_name(err));
        return 1;
    }

    char hex[65];
    usb_cli_ota_hex(actual_sha, sizeof(actual_sha), hex);
    printf("OTA DONE %u bytes sha=%s -> %s\n", (unsigned)received, hex,
           next->label);
    printf("Reboot to run the new image.\n");
    return 0;
}

void usb_cli_ota_register(void)
{
    const esp_console_cmd_t cmd_ota = {
        .command = "ota",
        .help    = "Firmware update over this console: ota <size> [sha256], then stream the raw image bytes",
        .hint    = "<size_bytes> [sha256_hex]",
        .func    = &usb_cli_cmd_ota,
    };

    (void)cmdline_manager_register(&cmd_ota);
}

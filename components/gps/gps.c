#include "gps.h"
#include "agnss_internal.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_console.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "config_manager.h"
#include "hw_config.h"

/* Internal AGNSS init — implemented in agnss.c (same component) */
esp_err_t agnss_init(bool enabled);

#include <nmea.h>
#include <gpgga.h>
#include <gprmc.h>

typedef struct
{
    bool has_fix;
    bool valid;
    double lat;
    double lon;
    int satellites;
    int fix_quality;
    double altitude_m;
    double speed_knots;
    double course_deg;
    uint32_t last_update_ms;
} gps_fix_t;

static bool s_inited = false;
static bool s_cmd_registered = false;

static volatile bool s_streaming = false;

static SemaphoreHandle_t s_fix_lock;
static gps_fix_t s_fix;

/* Queue for $PAIR response lines intercepted by the parser task.
 * Each item is a NUL-terminated string copied into a fixed-size buffer. */
#define PAIR_RESP_QUEUE_LEN  4
#define PAIR_RESP_MAX_LEN    160
static QueueHandle_t s_pair_queue;

static double gps_coord_to_decimal(int degrees, double minutes, char cardinal)
{
    double v = (double)degrees + (minutes / 60.0);
    if (cardinal == 'S' || cardinal == 'W')
    {
        v = -v;
    }
    return v;
}

static void gps_fix_update_from_gpgga(const nmea_gpgga_s *gpgga)
{
    if (!gpgga)
    {
        return;
    }

    if (xSemaphoreTake(s_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return;
    }

    s_fix.has_fix = true;
    s_fix.fix_quality = (int)gpgga->position_fix;
    s_fix.satellites = gpgga->n_satellites;
    s_fix.altitude_m = (double)gpgga->altitude;
    s_fix.lat = gps_coord_to_decimal(gpgga->latitude.degrees, gpgga->latitude.minutes, (char)gpgga->latitude.cardinal);
    s_fix.lon = gps_coord_to_decimal(gpgga->longitude.degrees, gpgga->longitude.minutes, (char)gpgga->longitude.cardinal);
    s_fix.last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    xSemaphoreGive(s_fix_lock);
}

static void gps_fix_update_from_gprmc(const nmea_gprmc_s *gprmc)
{
    if (!gprmc)
    {
        return;
    }

    if (xSemaphoreTake(s_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return;
    }

    s_fix.has_fix = true;
    s_fix.valid = gprmc->valid;
    s_fix.lat = gps_coord_to_decimal(gprmc->latitude.degrees, gprmc->latitude.minutes, (char)gprmc->latitude.cardinal);
    s_fix.lon = gps_coord_to_decimal(gprmc->longitude.degrees, gprmc->longitude.minutes, (char)gprmc->longitude.cardinal);
    s_fix.speed_knots = gprmc->gndspd_knots;
    s_fix.course_deg = gprmc->track_deg;
    s_fix.last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    xSemaphoreGive(s_fix_lock);
}

static esp_err_t gps_uart_setup_once(void)
{
    // Power on GPS module (best-effort)
    if (GPS_PWR_EN_PIN >= 0)
    {
        gpio_reset_pin(GPS_PWR_EN_PIN);
        gpio_set_direction(GPS_PWR_EN_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(GPS_PWR_EN_PIN, 1);
    }

    uart_config_t cfg = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(GPS_UART_PORT, 8192, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    err = uart_param_config(GPS_UART_PORT, &cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_set_pin(GPS_UART_PORT, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}

static void gps_parser_task(void *arg)
{
    (void)arg;

    uint8_t buf[256];
    char sentence[128];
    size_t sidx = 0;

    while (true)
    {
        int n = uart_read_bytes(GPS_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n <= 0)
        {
            continue;
        }

        for (int i = 0; i < n; i++)
        {
            const char c = (char)buf[i];

            if (sidx + 1 < sizeof(sentence))
            {
                sentence[sidx++] = c;
            }

            if (c == '\n')
            {
                sentence[sidx] = '\0';

                if (s_streaming)
                {
                    // Stream raw NMEA to console output.
                    // (stdout is redirected by usb_cli_console VFS)
                    printf("%s", sentence);
                }

                /* Intercept $PAIR responses and route to the queue */
                if (strncmp(sentence, "$PAIR", 5) == 0)
                {
                    if (s_pair_queue)
                    {
                        char qbuf[PAIR_RESP_MAX_LEN];
                        /* Strip trailing CR/LF for easier parsing */
                        size_t cl = sidx;
                        while (cl > 0 && (sentence[cl - 1] == '\r' ||
                                          sentence[cl - 1] == '\n'))
                        {
                            cl--;
                        }
                        if (cl >= sizeof(qbuf))
                        {
                            cl = sizeof(qbuf) - 1;
                        }
                        memcpy(qbuf, sentence, cl);
                        qbuf[cl] = '\0';
                        xQueueSend(s_pair_queue, qbuf, 0);
                    }
                }

                nmea_s *msg = nmea_parse(sentence, sidx, 0);
                if (msg)
                {
                    if (msg->type == NMEA_GPGGA)
                    {
                        gps_fix_update_from_gpgga((const nmea_gpgga_s *)msg);
                    }
                    else if (msg->type == NMEA_GPRMC)
                    {
                        gps_fix_update_from_gprmc((const nmea_gprmc_s *)msg);
                    }
                    nmea_free(msg);
                }

                sidx = 0;
            }
        }
    }
}

esp_err_t gps_init(bool enable_agnss)
{
    if (s_inited)
    {
        return ESP_OK;
    }

    if (!s_fix_lock)
    {
        s_fix_lock = xSemaphoreCreateMutex();
        if (!s_fix_lock)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = gps_uart_setup_once();
    if (err != ESP_OK)
    {
        return err;
    }

    if (!s_pair_queue)
    {
        s_pair_queue = xQueueCreate(PAIR_RESP_QUEUE_LEN,
                                    PAIR_RESP_MAX_LEN);
    }

    xTaskCreate(gps_parser_task, "gps_parser", 4096, NULL, 4, NULL);

    s_inited = true;

    /* Start AGNSS assistance if requested */
    agnss_init(enable_agnss);

    return ESP_OK;
}

esp_err_t gps_pair_response_wait(char *out, size_t out_len, uint32_t timeout_ms)
{
    if (!out || out_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_pair_queue)
    {
        return ESP_ERR_INVALID_STATE;
    }

    char qbuf[PAIR_RESP_MAX_LEN];
    if (xQueueReceive(s_pair_queue, qbuf, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
    {
        strncpy(out, qbuf, out_len - 1);
        out[out_len - 1] = '\0';
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t gps_uart_write(const void *data, size_t len)
{
    int w = uart_write_bytes(GPS_UART_PORT, data, len);
    uart_wait_tx_done(GPS_UART_PORT, pdMS_TO_TICKS(100));
    return (w == (int)len) ? ESP_OK : ESP_FAIL;
}

void gps_set_streaming(bool enabled)
{
    s_streaming = enabled;
}

bool gps_is_streaming(void)
{
    return s_streaming;
}

static void gps_print_last_fix(bool json)
{
    gps_fix_t snap = {0};

    if (xSemaphoreTake(s_fix_lock, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        snap = s_fix;
        xSemaphoreGive(s_fix_lock);
    }

    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    bool agnss_enabled = false;
    config_get_bool("AGNSS_ENABLED", &agnss_enabled);

    bool time_inj = false, pos_inj = false;
    agnss_get_status(&time_inj, &pos_inj);

    double cached_lat = 0, cached_lon = 0, cached_alt = 0;
    bool has_cached = (agnss_nvs_read_position(&cached_lat, &cached_lon,
                                               &cached_alt) == ESP_OK);

    if (!snap.has_fix)
    {
        if (json)
        {
            printf("{\"valid\":false,\"has_fix\":false,"
                   "\"agnss_enabled\":%s,\"time_injected\":%s,"
                   "\"position_injected\":%s",
                   agnss_enabled ? "true" : "false",
                   time_inj ? "true" : "false",
                   pos_inj ? "true" : "false");
            if (has_cached)
            {
                printf(",\"cached_lat\":%.6f,\"cached_lon\":%.6f,"
                       "\"cached_alt\":%.1f",
                       cached_lat, cached_lon, cached_alt);
            }
            printf("}\n");
        }
        else
        {
            printf("No GPS fix parsed yet.\n");
        }
        return;
    }

    uint32_t age_ms = now_ms - snap.last_update_ms;

    if (json)
    {
        printf("{\"valid\":%s,\"lat\":%.7f,\"lon\":%.7f,"
               "\"satellites\":%d,\"fix_quality\":%d,"
               "\"altitude_m\":%.1f,\"speed_knots\":%.2f,"
               "\"course_deg\":%.2f,\"age_ms\":%u,"
               "\"agnss_enabled\":%s,\"time_injected\":%s,"
               "\"position_injected\":%s",
               snap.valid ? "true" : "false",
               snap.lat, snap.lon,
               snap.satellites, snap.fix_quality,
               snap.altitude_m, snap.speed_knots,
               snap.course_deg, (unsigned)age_ms,
               agnss_enabled ? "true" : "false",
               time_inj ? "true" : "false",
               pos_inj ? "true" : "false");
        if (has_cached)
        {
            printf(",\"cached_lat\":%.6f,\"cached_lon\":%.6f,"
                   "\"cached_alt\":%.1f",
                   cached_lat, cached_lon, cached_alt);
        }
        printf("}\n");
    }
    else
    {
        printf(
            "valid=%d lat=%.7f lon=%.7f sat=%d fixq=%d alt=%.1fm spd=%.2fkn crs=%.2fdeg age_ms=%u\n",
            snap.valid,
            snap.lat,
            snap.lon,
            snap.satellites,
            snap.fix_quality,
            snap.altitude_m,
            snap.speed_knots,
            snap.course_deg,
            (unsigned)age_ms);
    }
}

// Weak symbol implemented by usb_cli_console so GPS component can switch input mode.
__attribute__((weak)) void usb_cli_console_set_gps_stream_mode(bool enabled)
{
    (void)enabled;
}

esp_err_t gps_get_fix(gps_fix_snapshot_t *out)
{
    if (!out)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_fix_lock)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    out->has_fix        = s_fix.has_fix;
    out->valid          = s_fix.valid;
    out->lat            = s_fix.lat;
    out->lon            = s_fix.lon;
    out->satellites     = s_fix.satellites;
    out->fix_quality    = s_fix.fix_quality;
    out->altitude_m     = s_fix.altitude_m;
    out->speed_knots    = s_fix.speed_knots;
    out->course_deg     = s_fix.course_deg;
    out->last_update_ms = s_fix.last_update_ms;

    xSemaphoreGive(s_fix_lock);

    return out->has_fix ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static int cmd_gps(int argc, char **argv)
{
    bool opt_stream = false;
    bool opt_print = false;
    bool opt_json = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-s") == 0)
        {
            opt_stream = true;
        }
        else if (strcmp(argv[i], "-p") == 0)
        {
            opt_print = true;
        }
        else if (strcmp(argv[i], "-j") == 0)
        {
            opt_json = true;
        }
        else
        {
            printf("Usage: gps -p [-j] | gps -s\n");
            return 1;
        }
    }

    if (opt_stream)
    {
        (void)gps_init(false);
        gps_set_streaming(true);
        usb_cli_console_set_gps_stream_mode(true);
        printf("GPS stream mode. Send +++++++ (or =======) + Enter to exit stream mode.\n");
        return 0;
    }

    if (!opt_print)
    {
        printf("Usage: gps -p [-j] | gps -s\n");
        return 1;
    }

    (void)gps_init(false);
    gps_print_last_fix(opt_json);
    return 0;
}

void gps_console_register(void)
{
    if (s_cmd_registered)
    {
        return;
    }

    const esp_console_cmd_t cmd = {
        .command = "gps",
        .help = "GPS helpers: gps -p [-j] prints last parsed fix (optional JSON), gps -s streams raw NMEA to console (exit with +++++++ or ======= + Enter)",
        .hint = NULL,
        .func = &cmd_gps,
    };

    (void)esp_console_cmd_register(&cmd);
    s_cmd_registered = true;
}

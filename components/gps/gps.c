#include "gps.h"

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

#include "hw_config.h"

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

esp_err_t gps_init(void)
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

    xTaskCreate(gps_parser_task, "gps_parser", 4096, NULL, 4, NULL);

    s_inited = true;
    return ESP_OK;
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

    if (!snap.has_fix)
    {
        printf("No GPS fix parsed yet.\n");
        return;
    }

    if (json)
    {
        printf(
            "{\"valid\":%s,\"lat\":%.7f,\"lon\":%.7f,\"satellites\":%d,\"fix_quality\":%d,\"altitude_m\":%.1f,\"speed_knots\":%.2f,\"course_deg\":%.2f,\"last_update_ms\":%u}\n",
            snap.valid ? "true" : "false",
            snap.lat,
            snap.lon,
            snap.satellites,
            snap.fix_quality,
            snap.altitude_m,
            snap.speed_knots,
            snap.course_deg,
            (unsigned)snap.last_update_ms);
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
            (unsigned)((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - snap.last_update_ms));
    }
}

// Weak symbol implemented by usb_cli_console so GPS component can switch input mode.
__attribute__((weak)) void usb_cli_console_set_gps_stream_mode(bool enabled)
{
    (void)enabled;
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
        (void)gps_init();
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

    (void)gps_init();
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

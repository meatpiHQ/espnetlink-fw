#include "gps.h"
#include "agnss_internal.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* IDF v6 removed soc/uart_channel.h; the UART0 default-pin macros live in
 * soc/uart_pins.h (U0TXD_GPIO_NUM/U0RXD_GPIO_NUM). */
#include "soc/uart_pins.h"

#include "hw_config.h"
#include "gps_internal.h"

/* Internal AGNSS init — implemented in agnss.c (same component) */
esp_err_t agnss_init(bool enabled);

#include <nmea.h>
#include <gpgga.h>
#include <gpgsa.h>
#include <gpgsv.h>
#include <gprmc.h>
#include <gpvtg.h>

typedef struct
{
    bool has_fix;
    bool valid;
    double lat;
    double lon;
    int satellites;
    int sats_in_view;
    int fix_quality;
    int fix_type;
    double altitude_m;
    double hdop;
    double pdop;
    double vdop;
    double speed_knots;
    double speed_kmph;
    double course_deg;
    uint32_t last_update_ms;
} gps_fix_t;

static bool s_inited = false;

static volatile bool s_streaming_usb = false;
static volatile bool s_streaming_uart0 = false;
static bool s_uart0_ready = false;

#define GPS_UART_RX_BUFFER_SIZE   16384
#define GPS_UART0_RX_BUFFER_SIZE  256
#define GPS_PARSER_TASK_STACK     4096
#define GPS_PARSER_TASK_PRIO      8
#define GPS_UART0_BAUD_RATE       115200

static SemaphoreHandle_t s_fix_lock;
static gps_fix_t s_fix;

/* Queue for $PAIR response lines intercepted by the parser task.
 * Each item is a NUL-terminated string copied into a fixed-size buffer. */
#define PAIR_RESP_QUEUE_LEN  4
#define PAIR_RESP_MAX_LEN    160
static QueueHandle_t s_pair_queue;

/* Event group bit: set by parser task whenever UART data is received */
#define GPS_EVT_UART_RX  BIT0
static EventGroupHandle_t s_gps_evt;
static TaskHandle_t s_gps_parser_task;

esp_err_t gps_uart0_prepare(void)
{
    uart_config_t cfg = {
        .baud_rate = GPS_UART0_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (!uart_is_driver_installed(UART_NUM_0))
    {
        esp_err_t err = uart_driver_install(UART_NUM_0,
                                            GPS_UART0_RX_BUFFER_SIZE,
                                            0,
                                            0,
                                            NULL,
                                            0);
        if (err != ESP_OK)
        {
            s_uart0_ready = false;
            return err;
        }
    }

    esp_err_t err = uart_param_config(UART_NUM_0, &cfg);
    if (err != ESP_OK)
    {
        s_uart0_ready = false;
        return err;
    }

    err = uart_set_pin(UART_NUM_0,
                       U0TXD_GPIO_NUM,
                       U0RXD_GPIO_NUM,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        s_uart0_ready = false;
        return err;
    }

    s_uart0_ready = true;
    return ESP_OK;
}

static void gps_uart0_write_sentence(const char *data, size_t len)
{
    if (!data || len == 0)
    {
        return;
    }

    if (!s_uart0_ready && gps_uart0_prepare() != ESP_OK)
    {
        return;
    }

    int written = uart_write_bytes(UART_NUM_0, data, len);
    if (written < 0)
    {
        s_uart0_ready = false;
        return;
    }

    (void)uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(20));
}

static void gps_apply_persisted_uart0_output(void)
{
    bool enabled = false;
    if (gps_settings_is_configured())
    {
        enabled = gps_settings()->uart0_out;
        if (enabled)
        {
            enabled = (gps_uart0_prepare() == ESP_OK);
        }
        s_streaming_uart0 = enabled;
    }
}

void gps_set_uart0_output(bool enabled)
{
    if (enabled)
    {
        if (gps_uart0_prepare() != ESP_OK)
        {
            s_streaming_uart0 = false;
            return;
        }
    }
    s_streaming_uart0 = enabled;
}

bool gps_is_uart0_output_enabled(void)
{
    return s_streaming_uart0;
}

esp_err_t gps_power_pin_set(bool enabled)
{
    esp_err_t err = gpio_reset_pin(GPS_PWR_EN_PIN);
    if (err != ESP_OK)
    {
        return err;
    }

    err = gpio_set_direction(GPS_PWR_EN_PIN, GPIO_MODE_OUTPUT);
    if (err != ESP_OK)
    {
        return err;
    }

    return gpio_set_level(GPS_PWR_EN_PIN, enabled ? 1 : 0);
}

bool gps_power_pin_is_enabled(void)
{
    return gpio_get_level(GPS_PWR_EN_PIN) != 0;
}

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

static void gps_fix_update_from_gpgsa(const nmea_gpgsa_s *gsa)
{
    if (!gsa)
    {
        return;
    }

    if (xSemaphoreTake(s_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return;
    }

    s_fix.fix_type = gsa->fixtype - '0';   /* '1'→1, '2'→2, '3'→3 */
    s_fix.pdop = gsa->pdop;
    s_fix.hdop = gsa->hdop;
    s_fix.vdop = gsa->vdop;

    xSemaphoreGive(s_fix_lock);
}

static void gps_fix_update_from_gpgsv(const nmea_gpgsv_s *gsv)
{
    if (!gsv)
    {
        return;
    }

    if (xSemaphoreTake(s_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return;
    }

    s_fix.sats_in_view = (int)gsv->satellites;

    xSemaphoreGive(s_fix_lock);
}

static void gps_fix_update_from_gpvtg(const nmea_gpvtg_s *vtg)
{
    if (!vtg)
    {
        return;
    }

    if (xSemaphoreTake(s_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return;
    }

    s_fix.speed_kmph = vtg->gndspd_kmph;

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

    esp_err_t err = uart_driver_install(GPS_UART_PORT, GPS_UART_RX_BUFFER_SIZE,
                                        0, 0, NULL, ESP_INTR_FLAG_IRAM);
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

        if (s_gps_evt)
        {
            xEventGroupSetBits(s_gps_evt, GPS_EVT_UART_RX);
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

                if (s_streaming_usb)
                {
                    // Stream raw NMEA to console output.
                    // (stdout is redirected by usb_cli_console VFS)
                    printf("%s", sentence);
                }

                if (s_streaming_uart0)
                {
                    gps_uart0_write_sentence(sentence, sidx);
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
                    else if (msg->type == NMEA_GPGSA)
                    {
                        gps_fix_update_from_gpgsa((const nmea_gpgsa_s *)msg);
                    }
                    else if (msg->type == NMEA_GPGSV)
                    {
                        gps_fix_update_from_gpgsv((const nmea_gpgsv_s *)msg);
                    }
                    else if (msg->type == NMEA_GPVTG)
                    {
                        gps_fix_update_from_gpvtg((const nmea_gpvtg_s *)msg);
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
        gps_apply_persisted_uart0_output();
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

    if (!s_gps_evt)
    {
        s_gps_evt = xEventGroupCreate();
    }

    /* §2: static stack in PSRAM (parser does UART + NMEA only — no flash
     * writes). TCB stays internal (FreeRTOS object). */
    static StaticTask_t s_parser_tcb;
    static StackType_t s_parser_stack[GPS_PARSER_TASK_STACK] EXT_RAM_BSS_ATTR;

#if CONFIG_FREERTOS_UNICORE
    s_gps_parser_task = xTaskCreateStatic(gps_parser_task, "gps_parser",
                                          GPS_PARSER_TASK_STACK, NULL,
                                          GPS_PARSER_TASK_PRIO,
                                          s_parser_stack, &s_parser_tcb);
#else
    s_gps_parser_task = xTaskCreateStaticPinnedToCore(
        gps_parser_task, "gps_parser", GPS_PARSER_TASK_STACK, NULL,
        GPS_PARSER_TASK_PRIO, s_parser_stack, &s_parser_tcb, 1);
#endif
    if (s_gps_parser_task == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_inited = true;

    /* Start AGNSS assistance if requested */
    agnss_init(enable_agnss);

    gps_apply_persisted_uart0_output();

    return ESP_OK;
}

bool gps_is_functional(uint32_t timeout_ms)
{
    if (!s_gps_evt)
    {
        return false;
    }

    /* Clear the bit, then wait for the parser task to set it again */
    xEventGroupClearBits(s_gps_evt, GPS_EVT_UART_RX);
    EventBits_t bits = xEventGroupWaitBits(s_gps_evt, GPS_EVT_UART_RX,
                                           pdTRUE, pdTRUE,
                                           pdMS_TO_TICKS(timeout_ms));
    return (bits & GPS_EVT_UART_RX) != 0;
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
    s_streaming_usb = enabled;
}

bool gps_is_streaming(void)
{
    return s_streaming_usb || s_streaming_uart0;
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
    out->sats_in_view   = s_fix.sats_in_view;
    out->fix_quality    = s_fix.fix_quality;
    out->fix_type       = s_fix.fix_type;
    out->altitude_m     = s_fix.altitude_m;
    out->hdop           = s_fix.hdop;
    out->pdop           = s_fix.pdop;
    out->vdop           = s_fix.vdop;
    out->speed_knots    = s_fix.speed_knots;
    out->speed_kmph     = s_fix.speed_kmph;
    out->course_deg     = s_fix.course_deg;
    out->last_update_ms = s_fix.last_update_ms;

    xSemaphoreGive(s_fix_lock);

    return out->has_fix ? ESP_OK : ESP_ERR_INVALID_STATE;
}


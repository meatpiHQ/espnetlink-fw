/**
 * @file main_cli_system.c
 * @brief Composite system console commands owned by the composition root
 *        (§6b): `ver`, `debug`, `system`, `reboot`, `sleep`, `wake`. Moved
 *        verbatim from usb_cli_console/usb_cli_commands.c — outputs are
 *        byte-identical.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "hw_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cmdline_manager.h"
#include "restart_tracker.h"

#include "gps.h"
#include "imu_manager.h"
#include "lte_upstream_pppos.h"
#include "usb_cli_console.h"

#include "main_compose.h"

static bool s_debug_enabled = false;

// ---------------------------------------------------------------------------
// ver / debug / system / reboot
// ---------------------------------------------------------------------------

static int cmd_ver(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("ESPNetLink USB CLI (CDC-ACM)\n");
    printf("FW %s\n", GIT_SHA);
    printf("ESP-IDF %s\n", esp_get_idf_version());
    return 0;
}

static int cmd_debug(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("debug %d\n", s_debug_enabled ? 1 : 0);
        return 0;
    }

    const int v = atoi(argv[1]);
    if (v == 0)
    {
        s_debug_enabled = false;
        esp_log_level_set("*", ESP_LOG_NONE);
        printf("debug 0\n");
        return 0;
    }
    if (v == 1)
    {
        s_debug_enabled = true;
        esp_log_level_set("*", ESP_LOG_DEBUG);
        printf("debug 1\n");
        return 0;
    }

    printf("Usage: debug 0|1\n");
    return 1;
}

static int cmd_system(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: system -v | -i | -s <unix_timestamp>\n");
        return 1;
    }

    if (strcmp(argv[1], "-s") == 0)
    {
        if (argc < 3)
        {
            printf("Usage: system -s <unix_timestamp>\n");
            return 1;
        }
        long ts = atol(argv[2]);
        if (ts <= 0)
        {
            printf("Invalid timestamp\n");
            return 1;
        }
        struct timeval tv = { .tv_sec = ts, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        lte_upstream_pppos_mark_time_synced();

        struct tm utc;
        time_t now = ts;
        gmtime_r(&now, &utc);
        printf("System time set to %04d-%02d-%02d %02d:%02d:%02d UTC\n",
               utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
               utc.tm_hour, utc.tm_min, utc.tm_sec);

        agnss_try_inject_time();
        return 0;
    }

    if (strcmp(argv[1], "-v") == 0)
    {
        printf("espnetlink-fw_" GIT_SHA "\n");
        return 0;
    }

    if (strcmp(argv[1], "-i") == 0)
    {
        esp_chip_info_t chip;
        esp_chip_info(&chip);

        const char *model_str;
        switch (chip.model)
        {
            case CHIP_ESP32S3: model_str = "ESP32-S3"; break;
            case CHIP_ESP32S2: model_str = "ESP32-S2"; break;
            case CHIP_ESP32C3: model_str = "ESP32-C3"; break;
            case CHIP_ESP32:   model_str = "ESP32";    break;
            default:           model_str = "Unknown";  break;
        }

        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        printf("FW version   : espnetlink-fw_" GIT_SHA "\n");
        printf("ESP-IDF      : %s\n", esp_get_idf_version());
        printf("Chip model   : %s rev %d\n", model_str, chip.revision);
        printf("CPU cores    : %d\n", chip.cores);
        printf("Free heap    : %lu bytes\n", (unsigned long)esp_get_free_heap_size());
        printf("Min free heap: %lu bytes\n", (unsigned long)esp_get_minimum_free_heap_size());
        printf("MAC (STA)    : %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return 0;
    }

    printf("Usage: system -v | -i | -s <unix_timestamp>\n");
    return 1;
}

static int cmd_reboot(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("Rebooting...\n");
    /* Cleanly exit CMUX/PPP so the modem returns to AT command mode.
     * Without this, the modem stays in CMUX after ESP restart and
     * modem_mgr_init has to power-cycle it (killing LTE registration). */
    (void)lte_upstream_pppos_stop();
    printf("\n\n\n");
    vTaskDelay(pdMS_TO_TICKS(1000));  // allow message to flush before reboot
    restart_tracker_restart(RESTART_TRACKER_PLANNED_REASON_USER_REQUEST,
                            RESTART_TRACKER_SOURCE_CONSOLE, 0);
    return 0;
}

// ---------------------------------------------------------------------------
// Sleep / Wake
// ---------------------------------------------------------------------------

static volatile bool s_sleep_mode_active = false;
static volatile bool s_deep_sleep_pending = false;
static volatile int s_deep_sleep_mode = 0;
static volatile uint8_t s_deep_sleep_wom_threshold = 50;

enum {
    DEEP_SLEEP_MODE_NONE = 0,
    DEEP_SLEEP_MODE_BASIC = 2,
    DEEP_SLEEP_MODE_WAKE_GPIO = 3,
    DEEP_SLEEP_MODE_IMU_WOM = 4,
};

static esp_err_t sleep_reset_wakeup_sources(void)
{
    esp_err_t err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (err != ESP_OK)
    {
        printf("Failed to clear wakeup sources: %s\n", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static void usb_shutdown_and_deep_sleep(void)
{
    printf("Sleeping now.\n\n\n");
    vTaskDelay(pdMS_TO_TICKS(300));  // flush USB output

    // Deep sleep powers the USB block down automatically.
    // Explicit USB teardown here can panic because the console stack is still active.
    esp_rom_printf(">>> entering deep sleep\n");
    esp_deep_sleep_start();
    // Should never reach here
    esp_rom_printf(">>> deep sleep FAILED!\n");
}

static void sleep_power_off_peripherals(void)
{
    // Stop LTE PPPoS stack and power off modem
    printf("Stopping LTE stack...\n");
    (void)lte_upstream_pppos_stop();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Ensure STATUS pin is configured as input before reading
    gpio_reset_pin(LTE_STATUS_PIN);
    gpio_set_direction(LTE_STATUS_PIN, GPIO_MODE_INPUT);

    // Ensure PWR_KEY is configured as output
    gpio_reset_pin(LTE_PWR_KEY_PIN);
    gpio_set_direction(LTE_PWR_KEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LTE_PWR_KEY_PIN, 0);

    // Check if modem is already off
    if (gpio_get_level(LTE_STATUS_PIN) != 0)
    {
        printf("LTE modem already OFF\n");
    }
    else
    {
        // Graceful modem power-off: 1500ms PWR_KEY pulse
        printf("Powering off LTE modem...\n");
        gpio_set_level(LTE_PWR_KEY_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1500));
        gpio_set_level(LTE_PWR_KEY_PIN, 0);

        // Wait for STATUS pin to go HIGH (modem OFF), up to 30s
        for (int i = 0; i < 60; i++)
        {
            if (gpio_get_level(LTE_STATUS_PIN) != 0)
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        printf("LTE modem %s\n",
               gpio_get_level(LTE_STATUS_PIN) != 0 ? "OFF" : "still on (timeout)");
    }

    // Power off GPS module
    printf("Powering off GPS...\n");
    gpio_set_level(GPS_PWR_EN_PIN, 0);

    // Turn off LEDs
    gpio_set_level(LED_RED_PIN, 1);  // active-low: HIGH = OFF

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
}

static esp_err_t prepare_mode_specific_wakeup(void)
{
    if (s_deep_sleep_mode == DEEP_SLEEP_MODE_WAKE_GPIO)
    {
        gpio_set_direction(USB_SEL_1, GPIO_MODE_OUTPUT);
        gpio_set_level(USB_SEL_1, 0);
        gpio_sleep_set_pull_mode(USB_SEL_1, GPIO_PULLDOWN_ONLY);
        gpio_pulldown_en(USB_SEL_1);
        rtc_gpio_pulldown_en(USB_SEL_1);
        gpio_hold_en(USB_SEL_1);

        rtc_gpio_init(WAKE_GPIO);
        rtc_gpio_set_direction(WAKE_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis(WAKE_GPIO);
        rtc_gpio_pullup_en(WAKE_GPIO);
        return esp_sleep_enable_ext1_wakeup_io(1ULL << WAKE_GPIO,
                                               ESP_EXT1_WAKEUP_ANY_LOW);
    }

    if (s_deep_sleep_mode == DEEP_SLEEP_MODE_IMU_WOM)
    {
        esp_rom_printf(">>> configuring WOM (%u mg)\n", (unsigned)s_deep_sleep_wom_threshold);

        /* imu_manager (WiCAN port): one call stops the detector, arms WoM
         * on INT1 and drains latched statuses (was configure+prepare). */
        esp_err_t err =
            imu_manager_prepare_wom_wake(s_deep_sleep_wom_threshold);
        if (err != ESP_OK)
        {
            return err;
        }

        esp_rom_printf(">>> arming IMU EXT0 wake\n");
        rtc_gpio_init(IMU_INT_PIN);
        rtc_gpio_set_direction(IMU_INT_PIN, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_dis(IMU_INT_PIN);
        rtc_gpio_pulldown_en(IMU_INT_PIN);
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        return esp_sleep_enable_ext0_wakeup(IMU_INT_PIN, 1);
    }

    return ESP_OK;
}

static void deep_sleep_worker_task(void *arg)
{
    (void)arg;

    usb_cli_console_set_enabled(false);
    vTaskDelay(pdMS_TO_TICKS(250));

    sleep_power_off_peripherals();
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_rom_printf(">>> preparing wake source\n");
    esp_err_t err = prepare_mode_specific_wakeup();
    if (err != ESP_OK)
    {
        s_deep_sleep_pending = false;
        s_deep_sleep_mode = DEEP_SLEEP_MODE_NONE;
        esp_rom_printf(">>> deep sleep aborted: %s\n", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    usb_shutdown_and_deep_sleep();
    vTaskDelete(NULL);
}

static int start_deep_sleep_worker(int mode)
{
    if (s_deep_sleep_pending)
    {
        printf("Deep sleep is already pending\n");
        return 1;
    }

    s_deep_sleep_pending = true;
    s_deep_sleep_mode = mode;
    BaseType_t created = xTaskCreate(deep_sleep_worker_task,
                                     "deep_sleep",
                                     4096,
                                     NULL,
                                     9,
                                     NULL);
    if (created != pdPASS)
    {
        s_deep_sleep_pending = false;
        s_deep_sleep_mode = DEEP_SLEEP_MODE_NONE;
        printf("Failed to create deep sleep task\n");
        return 1;
    }

    return 0;
}

static int cmd_sleep(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: sleep <mode> [args]\n");
        printf("  sleep 1            - Light sleep: power off GPS/LTE, keep USB active\n");
        printf("                       Wake with 'wake' command\n");
        printf("  sleep 2 [secs]     - Deep sleep: power off everything including USB\n");
        printf("                       Wake on timer (if secs given) or reset button\n");
        printf("  sleep 3            - Deep sleep: USB mux off, wake on WAKE_GPIO\n");
        printf("  sleep 4 [thresh]   - Deep sleep: wake on motion (IMU)\n");
        printf("                       thresh = WOM threshold in mg (1-255, default 50)\n");
        return 1;
    }

    const int mode = atoi(argv[1]);

    if (mode == 1)
    {
        printf("Entering low-power mode (USB active)...\n");
        sleep_power_off_peripherals();
        s_sleep_mode_active = true;
        printf("Low-power mode active. Type 'wake' to restore.\n");
        return 0;
    }

    if (mode == 2)
    {
        uint64_t sleep_us = 0;
        if (sleep_reset_wakeup_sources() != ESP_OK)
        {
            return 1;
        }

        if (argc >= 3)
        {
            long secs = atol(argv[2]);
            if (secs <= 0)
            {
                printf("Invalid duration\n");
                return 1;
            }
            sleep_us = (uint64_t)secs * 1000000ULL;
            printf("Entering deep sleep for %ld seconds...\n", secs);
            esp_sleep_enable_timer_wakeup(sleep_us);
        }
        else
        {
            printf("Entering deep sleep (wake on reset only)...\n");
        }

        return start_deep_sleep_worker(DEEP_SLEEP_MODE_BASIC);
    }

    if (mode == 3)
    {
        printf("Entering deep sleep (USB mux off, wake on WAKE_GPIO)...\n");

        if (sleep_reset_wakeup_sources() != ESP_OK)
        {
            return 1;
        }

        return start_deep_sleep_worker(DEEP_SLEEP_MODE_WAKE_GPIO);
    }

    if (mode == 4)
    {
        uint8_t threshold = 50;  // default 50 mg
        if (argc >= 3)
        {
            int val = atoi(argv[2]);
            if (val < 1 || val > 255)
            {
                printf("Threshold must be 1-255 mg\n");
                return 1;
            }
            threshold = (uint8_t)val;
        }

        printf("Configuring IMU wake-on-motion (threshold=%u mg)...\n", threshold);
        if (sleep_reset_wakeup_sources() != ESP_OK)
        {
            return 1;
        }

        printf("Entering deep sleep (wake on motion)...\n");

        s_deep_sleep_wom_threshold = threshold;

        return start_deep_sleep_worker(DEEP_SLEEP_MODE_IMU_WOM);
    }

    printf("Unknown mode %d. Use 1, 2, 3, or 4.\n", mode);
    return 1;
}

static int cmd_wake(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (!s_sleep_mode_active)
    {
        printf("Device is not in sleep mode\n");
        return 1;
    }

    printf("Waking up — rebooting to re-initialize peripherals...\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    restart_tracker_restart(RESTART_TRACKER_PLANNED_REASON_POWER_WAKE,
                            RESTART_TRACKER_SOURCE_CONSOLE, 0);
    return 0;
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

esp_err_t main_cli_register_system(void)
{
    static const esp_console_cmd_t CMDS[] =
    {
        { .command = "ver",
          .help    = "Print firmware/version info",
          .hint    = NULL,
          .func    = &cmd_ver },
        { .command = "debug",
          .help    = "Enable/disable global log output (0=off, 1=on)",
          .hint    = NULL,
          .func    = &cmd_debug },
        { .command = "system",
          .help    = "System info/control: -v (version), -i (device info), -s <ts> (set time)",
          .hint    = "-v | -i | -s <unix_ts>",
          .func    = &cmd_system },
        { .command = "reboot",
          .help    = "Reboot the device",
          .hint    = NULL,
          .func    = &cmd_reboot },
        { .command = "sleep",
          .help    = "Enter low-power mode. 1=USB active, 2=deep sleep (timer/reset), 3=deep sleep (GPIO wake)",
          .hint    = "<1|2|3> [seconds]",
          .func    = &cmd_sleep },
        { .command = "wake",
          .help    = "Wake from sleep mode 1 (reboots to re-init peripherals)",
          .hint    = NULL,
          .func    = &cmd_wake },
    };

    esp_err_t err = ESP_OK;

    for (size_t i = 0; i < sizeof(CMDS) / sizeof(CMDS[0]) && err == ESP_OK;
         i++)
    {
        err = cmdline_manager_register(&CMDS[i]);
    }

    return err;
}

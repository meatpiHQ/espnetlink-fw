/*
 * Main application (C) using usb_cdc_host_manager to manage CH342 interfaces 0 and 2 separately
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "hw_config.h"
#include "filesystem.h"
#include "gps.h"
#include "imu.h"
#include "usb_cli_console.h"

#include "main_compose.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_attr.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"

#include "cJSON.h"

#include "api_http.h"
#include "cmdline_manager.h"
#include "dev_status_manager.h"
#include "http_server_manager.h"
#include "log_manager.h"
#include "settings_manager.h"
#include "wifi_manager.h"

// Select exactly one mode (set one to 1, the other to 0)
#define USB_HOST_MODE        0
#define USB_DEV_MODE         0
#define USB_DEV_ETHERNET_MODE 1

#if USB_HOST_MODE
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#include "usb_cdc_host_manager.h"
#include "usb_cdc_uart_bridge.h"
#include "modem_manager.h"
#endif

#if USB_DEV_MODE
#include "usb_dev_bridge.h"
#endif

#if USB_DEV_ETHERNET_MODE
#include "usb_dev_ethernet.h"
#endif

// Change these values to match your needs
#define EXAMPLE_BAUDRATE     (2000000)
#define EXAMPLE_STOP_BITS    (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY       (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS    (8)

static const char *TAG = "main";

#if USB_DEV_ETHERNET_MODE
// Optional: bring up BG95 PPPoS upstream and share it over USB-NCM.
// (No Kconfig use here; just flip this macro.)
#define ENABLE_LTE_UPSTREAM_PPPOS  1

#if ENABLE_LTE_UPSTREAM_PPPOS
#include "lte_upstream_pppos.h"

static void lte_upstream_task(void *arg)
{
    (void)arg;

    /* Boot-applied LTE configuration (settings_manager descriptor
     * "lte_upstream_pppos", reboot-to-apply §4.2). */
    const lte_upstream_pppos_settings_t *lte = lte_upstream_pppos_settings();

    if (!lte_upstream_pppos_settings_is_configured() || !lte->enabled)
    {
        ESP_LOGI(TAG, "LTE disabled in config – skipping LTE upstream");
        vTaskDelete(NULL);
        return;
    }

    bool ncm_sharing = lte->ncm_share;

    ESP_LOGI(TAG, "LTE config: APN='%s' user='%s' pin=%s",
             lte->apn,
             lte->apn_user[0] ? lte->apn_user : "(none)",
             lte->apn_pin[0]  ? "(set)"       : "(none)");

    const lte_upstream_pppos_config_t cfg = {
        .modem = {
            .uart_port = LTE_UART_PORT,
            .tx_pin = LTE_UART_TX_PIN,
            .rx_pin = LTE_UART_RX_PIN,
            .rts_pin = LTE_RTS_PIN,
            .cts_pin = LTE_CTS_PIN,
            .pwr_key_pin = LTE_PWR_KEY_PIN,
            .status_pin = LTE_STATUS_PIN,
            .dtr_pin = LTE_DTR_PIN,
            .status_on_is_low = true,
        },

        .apn  = lte->apn,
        .user = lte->apn_user[0]     ? lte->apn_user     : NULL,
        .pass = lte->apn_password[0] ? lte->apn_password : NULL,
        .pin  = lte->apn_pin[0]      ? lte->apn_pin      : NULL,

        .init_modem_manager = true,
        .baud_rate = 3000000,
        .hw_flow_control = true,

        .enable_usb_ncm_sharing = ncm_sharing,
        .connect_timeout_ms = 0,
        /* If the modem gets stuck failing registration, issue a soft reboot. */
        .reboot_on_reg_timeout = true,
        .reboot_wait_ms = 90000,
        .use_cmux = true,
    };

    ESP_LOGI(TAG, "Starting LTE upstream PPPoS...");
    esp_err_t err = lte_upstream_pppos_start(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "lte_upstream_pppos_start failed: %s", esp_err_to_name(err));
    }

    vTaskDelete(NULL);
}
#endif // ENABLE_LTE_UPSTREAM_PPPOS
#endif

#if USB_HOST_MODE
static SemaphoreHandle_t device_disconnected_sem;
#endif

// RX callbacks for each interface
#if USB_HOST_MODE
static bool handle_rx0(const uint8_t *data, size_t data_len, void *arg)
{
    ESP_LOGI(TAG, "RX iface0: %.*s", (int)data_len, (const char *)data);
    ESP_LOG_BUFFER_HEX(TAG, data, data_len);
    return true;
}

static void handle_event0(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error iface0 err_no=%d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device disconnected (iface0)");
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Iface0 serial state 0x%04X", event->data.serial_state.val);
        break;
    default: break;
    }
}

static bool handle_rx2(const uint8_t *data, size_t data_len, void *arg)
{
    ESP_LOGI(TAG, "RX iface2: %.*s", (int)data_len, (const char *)data);
    // ESP_LOG_BUFFER_HEX(TAG, data, data_len);
    return true;
}

static void handle_event2(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error iface2 err_no=%d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device disconnected (iface2)");
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Iface2 serial state 0x%04X", event->data.serial_state.val);
        break;
    default: break;
    }
}

static void iface_tx_task(void *arg)
{
    const uint8_t iface = (uint32_t)arg; // 0 or 2
    const char *cmd = "ATI\r";
    while (1) {
        ESP_LOGI(TAG, "Sending command on iface %u: %s", iface, cmd);
        ESP_ERROR_CHECK_WITHOUT_ABORT(usb_cdc_mgr_tx(iface, (const uint8_t *)cmd, strlen(cmd), 1000));
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
#endif // USB_HOST_MODE

/* ---- cJSON heap -> PSRAM (standard §2; WiCAN main pattern) ------------------
 * Internal RAM is the scarce resource; JSON is the settings/API lingua franca
 * and its trees live long. cJSON is only touched from task context, so ALL of
 * it belongs in PSRAM. Falls back to the default heap when PSRAM is
 * exhausted. */
static void *cjson_psram_malloc(size_t n)
{
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    return (p != NULL) ? p : malloc(n);
}

static void cjson_psram_free(void *p)
{
    free(p); /* heap_caps pointers free through the same allocator */
}

/** Init/register one step. NEVER panics: a deterministic init failure under
 *  ESP_ERROR_CHECK would be a permanent boot loop — the one thing a field
 *  device must never do (standard §3). A failed init leaves that component
 *  unconfigured and the rest of the device keeps booting. */
static bool init_step(const char *name, esp_err_t (*fn)(void))
{
    esp_err_t err = fn();

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s failed: %s (continuing degraded)", name,
                 esp_err_to_name(err));
    }

    return err == ESP_OK;
}

/** Start one component; a failure degrades, never halts boot (§4.3). */
static bool start_component(const char *name, esp_err_t (*start_fn)(void))
{
    esp_err_t err = start_fn();

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s_start failed: %s (continuing degraded)", name,
                 esp_err_to_name(err));
    }

    return err == ESP_OK;
}

void app_main(void)
{
    /* BEFORE any component init: every cJSON allocation firmware-wide goes
     * through the PSRAM-first hooks. */
    static const cJSON_Hooks CJSON_PSRAM =
    {
        .malloc_fn = cjson_psram_malloc,
        .free_fn = cjson_psram_free,
    };

    cJSON_InitHooks((cJSON_Hooks *)&CJSON_PSRAM);

    // Log reset and wakeup reason at boot time, before any long startup delays.
    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGW("main", "Reset reason: %d", (int)reset_reason);
    if (wakeup != ESP_SLEEP_WAKEUP_UNDEFINED)
    {
        const char *reason = "unknown";
        switch (wakeup)
        {
        case ESP_SLEEP_WAKEUP_EXT0:      reason = "EXT0";      break;
        case ESP_SLEEP_WAKEUP_EXT1:      reason = "EXT1";      break;
        case ESP_SLEEP_WAKEUP_TIMER:     reason = "TIMER";     break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:  reason = "TOUCHPAD";  break;
        case ESP_SLEEP_WAKEUP_ULP:       reason = "ULP";       break;
        case ESP_SLEEP_WAKEUP_GPIO:      reason = "GPIO";      break;
        default: break;
        }
        ESP_LOGW("main", "Woke from deep sleep, cause: %s (%d)", reason, (int)wakeup);
    }


    /* Log pipeline first (§9.6): everything below needs somewhere to log.
     * dev_status early: the device id feeds the AP SSID suffix. */
    init_step("log_manager_init", log_manager_init);
    init_step("dev_status_manager_init", dev_status_manager_init);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ret = nvs_flash_erase();
        if (ret == ESP_OK)
        {
            ret = nvs_flash_init();
        }
    }
    if (ret != ESP_OK)
    {
        /* Settings live on their own LittleFS partition, not NVS — a dead
         * NVS mainly costs WiFi PHY cal + the AGNSS cache. Degrade. */
        ESP_LOGE(TAG, "nvs_flash_init failed: %s (continuing degraded)",
                 esp_err_to_name(ret));
    }
	gpio_reset_pin(USB_SEL_1);
	gpio_set_direction(USB_SEL_1, GPIO_MODE_INPUT);
	gpio_set_pull_mode(USB_SEL_1, GPIO_FLOATING);

	gpio_reset_pin(USB_SEL_2);
	gpio_set_direction(USB_SEL_2, GPIO_MODE_INPUT);
	gpio_set_pull_mode(USB_SEL_2, GPIO_FLOATING);

    gpio_reset_pin(BUTTON_GPIO);
	gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    if(gpio_get_level(BUTTON_GPIO) == 0)
    {
        ESP_LOGI(TAG, "BOOT button held at startup: entering flash mode");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (gpio_get_level(USB_SEL_2) == 1)
    {
        ESP_LOGI(TAG, "USB_SEL_2 is HIGH: checking modem state");
        /* STATUS pin is LOW when modem is ON (status_on_is_low = true) */
        gpio_reset_pin(LTE_STATUS_PIN);
        gpio_set_direction(LTE_STATUS_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode(LTE_STATUS_PIN, GPIO_FLOATING);

        gpio_reset_pin(LTE_PWR_KEY_PIN);
        gpio_set_direction(LTE_PWR_KEY_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(LTE_PWR_KEY_PIN, 0);

        if (gpio_get_level(LTE_STATUS_PIN) != 0)
        {
            /* Modem is OFF – pulse PWR_KEY HIGH for 800 ms to power it on */
            ESP_LOGI(TAG, "Modem is OFF – sending power-on pulse");
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LTE_PWR_KEY_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(800));
            gpio_set_level(LTE_PWR_KEY_PIN, 0);
            ESP_LOGI(TAG, "Modem power-on pulse complete");
        }
        else
        {
            ESP_LOGI(TAG, "Modem is already ON – skipping power-on pulse");
        }

        ESP_LOGI(TAG, "Entering infinite loop");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if(gpio_get_level(USB_SEL_1) == 0)
    {
        ESP_LOGI(TAG, "USB_SEL_1 is LOW: entering flash mode");
        while (gpio_get_level(USB_SEL_1) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
    }

    /* ---- MeatPi core platform (standard §3/§4/§9) ------------------------- */
    init_step("settings_manager_init", settings_manager_init);
    init_step("log_manager_register_settings", log_manager_register_settings);
    init_step("dev_status_manager_register_settings",
              dev_status_manager_register_settings);

    /* §9.2 exception: settings_manager can't register with log_manager
     * (cycle) — the composition root does it. */
    static const log_descriptor_t SETTINGS_LOG =
        { "settings_manager", ESP_LOG_INFO };

    log_manager_register(&SETTINGS_LOG);

    init_step("esp_netif_init", esp_netif_init);
    init_step("esp_event_loop_create_default", esp_event_loop_create_default);
    init_step("http_server_manager_init", http_server_manager_init);
    init_step("wifi_manager_init", wifi_manager_init);
    init_step("cmdline_manager_init", cmdline_manager_init);
    init_step("api_http_init", api_http_init);
    /* product components register their settings descriptors (§4.1);
     * their console commands register from the settings boot apply,
     * gated by each component's `cli` setting (§6b) */
    init_step("lte_upstream_pppos_register_settings",
              lte_upstream_pppos_register_settings);
    init_step("gps_register_settings", gps_register_settings);
    init_step("imu_register_settings", imu_register_settings);
    init_step("usb_cli_console_register_settings",
              usb_cli_console_register_settings);
    init_step("main_register_settings", main_register_settings);
    /* composite CLI commands owned by main (§6b): `config` */
    init_step("main_cli_register", main_cli_register);
    init_step("wifi_manager_register_http", wifi_manager_register_http);
    (void)http_server_manager_set_request_gate(
        wifi_manager_http_request_allowed);

    filesystem_init();

    /* config_manager retired: persistence lives in settings_manager (§4);
     * gps_init moved below the settings apply pass (its config now comes
     * from the "gps" descriptor). */

    // Initialize IMU (ICM42670) over I2C
    esp_err_t imu_err = imu_init();
    if (imu_err != ESP_OK)
    {
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(imu_err));
    }

    #if ((USB_HOST_MODE + USB_DEV_MODE + USB_DEV_ETHERNET_MODE) != 1)
    #error "Select exactly one of USB_HOST_MODE, USB_DEV_MODE, USB_DEV_ETHERNET_MODE"
    #elif USB_HOST_MODE
    //init modem manager here
    
    // Initialize BG95 modem (set 3M baud + HW flow control, ensure command mode)
    const modem_mgr_config_t modem_cfg = {
        .uart_port = LTE_UART_PORT,
        .tx_pin = LTE_UART_TX_PIN,
        .rx_pin = LTE_UART_RX_PIN,
        .rts_pin = LTE_RTS_PIN,
        .cts_pin = LTE_CTS_PIN,
        .pwr_key_pin = LTE_PWR_KEY_PIN,
        .status_pin = LTE_STATUS_PIN,
        .dtr_pin = LTE_DTR_PIN,
        .status_on_is_low = true, // BG95: STATUS low when ON
    };
    if (modem_mgr_init(&modem_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Modem init failed");
    }

    vTaskDelay(pdMS_TO_TICKS(15000));

    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Install host + CDC-ACM and start event task
    ESP_ERROR_CHECK(usb_cdc_mgr_init());



    ESP_ERROR_CHECK(usb_cdc_mgr_open(USB_VID_QINHENG, USB_PID_CH342, USB_CDC_IFACE_2,
                                     5000, 1024*15, 1024*15, handle_event2, handle_rx2, NULL));
    ESP_ERROR_CHECK(usb_cdc_mgr_set_line_coding(USB_CDC_IFACE_2,
                                                115200,
                                                EXAMPLE_STOP_BITS,
                                                EXAMPLE_PARITY,
                                                EXAMPLE_DATA_BITS));
    // Open interface 0
    // ESP_ERROR_CHECK(usb_cdc_mgr_open(USB_VID_QINHENG, USB_PID_CH342, USB_CDC_IFACE_0,
    //                                  5000, 5120, 5120, handle_event0, handle_rx0, NULL));
    // ESP_ERROR_CHECK(usb_cdc_mgr_set_line_coding(USB_CDC_IFACE_0,
    //                                             2000000,
    //                                             EXAMPLE_STOP_BITS,
    //                                             EXAMPLE_PARITY,
    //                                             EXAMPLE_DATA_BITS));


    // Start UART<->USB bridge: interface 0 <-> modem UART
    usb_cdc_uart_bridge_config_t br = {
        // UART (use LTE_* from hw_config.h)
        .uart_port = LTE_UART_PORT,
        .uart_tx_pin = LTE_UART_TX_PIN,
        .uart_rx_pin = LTE_UART_RX_PIN,
        .use_hw_flow = true,
        .uart_rts_pin = LTE_RTS_PIN,
        .uart_cts_pin = LTE_CTS_PIN,
        .uart_baud = 3000000,       // match modem_mgr target
        .uart_data_bits = 8,
        .uart_parity = 0,
        .uart_stop_bits = 1,

        // USB device selection (CH342 interface 0)
        .usb_vid = USB_VID_QINHENG,
        .usb_pid = USB_PID_CH342,
        .usb_interface = USB_CDC_IFACE_0,

        // USB line coding (match UART)
        .usb_baud = 3000000,
        .usb_data_bits = 8,
        .usb_parity = 0,
        .usb_stop_bits = 0,         // 0=1 stop bit

        // Buffers/timeouts
        .usb_conn_timeout_ms = 5000,
        .usb_tx_timeout_ms = 1000,
        .usb_out_buffer = 1024*15,
        .usb_in_buffer = 1024*15,

        // Optional taps (none)
        .event_cb = NULL,
        .rx_tap_cb = NULL,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(usb_cdc_uart_bridge_start(&br));
    #elif USB_DEV_MODE
    ESP_ERROR_CHECK(usb_dev_bridge_start());
    #elif USB_DEV_ETHERNET_MODE
    /* ---- the ONE settings apply pass (§4.3), then start the core --------- */
    init_step("settings_manager_start", settings_manager_start);
    init_step("log_manager_start", log_manager_start);

    /* one-time legacy config.json import (reboots once when it changes
     * anything, so field devices keep their configuration) */
    main_legacy_import_run();

    // GPS after the settings apply: its config is the "gps" descriptor
    esp_err_t gps_err = gps_init(gps_settings()->agnss_enabled);
    if (gps_err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPS init failed: %s", esp_err_to_name(gps_err));
    }

    start_component("wifi_manager", wifi_manager_start);
    start_component("http_server_manager", http_server_manager_start);
    start_component("cmdline_manager", cmdline_manager_start);

    usb_dev_ethernet_set_console_enabled(true);
    start_component("usb_dev_ethernet", usb_dev_ethernet_start);

#if USB_DEV_ETHERNET_MODE && ENABLE_LTE_UPSTREAM_PPPOS
    /* Pinned to core 1: uart_driver_install allocates its interrupt on the
     * calling core, and core 0's interrupt slots are exhausted now that
     * esp_wifi is composed (bench-hit 2026-07-11: esp_modem UART DTE
     * install failed with ESP_FAIL right after the wifi stack came up).
     * §2: static stack in PSRAM (config + esp_modem setup — no flash). */
    static StaticTask_t s_lte_tcb;
    static StackType_t s_lte_stack[6144] EXT_RAM_BSS_ATTR;

    (void)xTaskCreateStaticPinnedToCore(lte_upstream_task, "lte_pppos", 6144,
                                        NULL, 5, s_lte_stack, &s_lte_tcb, 1);
#endif
    #endif
    // Optionally, you can still use interface 0 independently as before
    //init cdc manager for other interfaces

    // Start TX tasks for each interface
    // xTaskCreate(iface_tx_task, "iface0_tx", 4096, (void *)USB_CDC_IFACE_0, 5, NULL);
    // xTaskCreate(iface_tx_task, "iface2_tx", 4096, (void *)USB_CDC_IFACE_2, 5, NULL);
}

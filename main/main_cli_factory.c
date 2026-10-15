/**
 * @file main_cli_factory.c
 * @brief `factory`: the production tester's device side (standard 6b: a
 *        composite spanning gps, imu_manager, led_manager, lte_upstream_pppos,
 *        wifi_manager, wifi_modem, filesystem, restart_tracker and the
 *        hw_config pins, so main owns it). tools/production/prodtest.py
 *        drives it over the CDC console; every answer is ONE JSON line so the
 *        host parses nothing else. Pass/fail thresholds live on the host.
 *
 *   factory selftest [-w <ssid>]           identity, memory, boot counters,
 *                                          USB mux readback, button level, IMU
 *                                          (id, accel, temp, INT1 trace), LED
 *                                          arbiter, GPS (rx, version, sats),
 *                                          storage, LTE stage/SIM; -w adds a
 *                                          WiFi scan reporting <ssid>'s RSSI
 *   factory button <ms>                    wait up to <ms> for a PB1 press and
 *                                          release
 *   factory usbmux esp|ch343|bg95|off <ms> route the USB connector for <ms>,
 *                                          then release the mux to the DIP
 *                                          route (the console drops meanwhile)
 *   factory led red 0|1                    the GPIO41 red LED (v1.60 boards)
 *   factory mark <serial> [station]        write /data/factory.json
 *   factory mark                           print /data/factory.json
 *   factory coredump [erase]               the panic stored in the coredump
 *                                          partition as one JSON line (task,
 *                                          cause, PC, backtrace; decode with
 *                                          addr2line against the build's ELF),
 *                                          or erase it
 *   factory gpio <n>                       the driver's live configuration of
 *                                          one pin plus its level (bench
 *                                          diagnosis)
 *   factory bootloader                     reboot into ROM download mode for
 *                                          an esptool erase or re-flash with
 *                                          the DIP left in the run position.
 *                                          The chip comes up as the ROM's
 *                                          USB-OTG download device
 *                                          (303A:0009, CDC + DFU), not as the
 *                                          serial-JTAG: the OTG stack's PHY
 *                                          routing lives in RTC_CNTL and
 *                                          survives the soft reset
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_core_dump.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "hw_config.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cJSON.h"
#include "cmdline_manager.h"
#include "dev_status_manager.h"
#include "usb_dev_ethernet.h"
#include "filesystem.h"
#include "gps.h"
#include "imu_manager.h"
#include "led_manager.h"
#include "lte_upstream_pppos.h"
#include "restart_tracker.h"
#include "wifi_manager.h"
#include "wifi_modem.h"

#include "main_compose.h"

#define FACTORY_RECORD_PATH        "/data/factory.json"
#define FACTORY_RECORD_MAX         512
#define FACTORY_FIELD_MAX          32

#define FACTORY_GPS_RX_WAIT_MS     2000
#define FACTORY_GPS_VER_WAIT_MS    3000

#define FACTORY_MUX_MIN_MS         500
#define FACTORY_MUX_MAX_MS         60000
#define FACTORY_MUX_ANSWER_MS      300  /* the answer reaches the host first  */
#define FACTORY_MUX_DETACH_MS      150  /* all-off between two routes         */

#define FACTORY_BUTTON_MIN_MS      100
#define FACTORY_BUTTON_MAX_MS      60000
#define FACTORY_BUTTON_POLL_MS     10
#define FACTORY_BUTTON_DEBOUNCE_MS 30

/* the whole history ring: keep it off the console task's stack (§2) */
static restart_tracker_state_t s_rt_state EXT_RAM_BSS_ATTR;

static void print_json(cJSON *root)
{
    char *s = cJSON_PrintUnformatted(root);

    if (s == NULL)
    {
        printf("{\"error\":\"out of memory\"}\n");
        return;
    }

    printf("%s\n", s);
    cJSON_free(s);
}

static double round_to(float v, float scale)
{
    return (double)roundf(v * scale) / (double)scale;
}

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/* ---- selftest sections ------------------------------------------------- */

static void add_identity(cJSON *root)
{
    esp_chip_info_t chip;
    uint8_t mac[6] = { 0 };
    char mac_str[18];

    esp_chip_info(&chip);
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    cJSON_AddStringToObject(root, "fw", dev_status_manager_app_version());
    cJSON_AddStringToObject(root, "idf", esp_get_idf_version());
    cJSON_AddStringToObject(root, "chip",
                            chip.model == CHIP_ESP32S3 ? "ESP32-S3" : "other");
    cJSON_AddNumberToObject(root, "chip_rev", chip.revision);
    cJSON_AddStringToObject(root, "mac", mac_str);
    cJSON_AddStringToObject(root, "device_id", dev_status_manager_device_id());
    cJSON_AddStringToObject(root, "partition",
                            dev_status_manager_partition_label());
    cJSON_AddStringToObject(root, "reset_reason",
                            restart_tracker_reset_reason_to_str(
                                (uint32_t)esp_reset_reason()));

    if (restart_tracker_get_state(&s_rt_state) == ESP_OK)
    {
        cJSON_AddNumberToObject(root, "boot_count", s_rt_state.boot_count);
        cJSON_AddNumberToObject(root, "unexpected_resets",
                                s_rt_state.unexpected_reset_count);
    }

    dev_status_memory_t mem;

    if (dev_status_manager_memory(&mem) == ESP_OK)
    {
        cJSON *m = cJSON_AddObjectToObject(root, "memory");

        cJSON_AddNumberToObject(m, "psram_total", mem.psram.total);
        cJSON_AddNumberToObject(m, "psram_free", mem.psram.free);
        cJSON_AddNumberToObject(m, "internal_free", mem.internal.free);
        cJSON_AddNumberToObject(m, "internal_min_free", mem.internal.min_free);
    }

    float temp_c = 0;

    if (dev_status_manager_temperature(&temp_c) == ESP_OK)
    {
        cJSON_AddNumberToObject(root, "die_temp_c", round_to(temp_c, 10.0f));
    }
}

static void add_pins(cJSON *root)
{
    cJSON *mux = cJSON_AddObjectToObject(root, "mux");

    cJSON_AddNumberToObject(mux, "sel1", gpio_get_level(USB_SEL_1));
    cJSON_AddNumberToObject(mux, "sel2", gpio_get_level(USB_SEL_2));
    cJSON_AddBoolToObject(mux, "usb_data", wifi_modem_usb_data_routed());

    cJSON_AddNumberToObject(root, "button_level", gpio_get_level(BUTTON_GPIO));
}

static void add_imu(cJSON *root)
{
    cJSON *o = cJSON_AddObjectToObject(root, "imu");
    uint8_t id = 0;
    esp_err_t err = imu_manager_device_id(&id);

    if (err != ESP_OK)
    {
        cJSON_AddBoolToObject(o, "ok", false);
        cJSON_AddStringToObject(o, "err", esp_err_to_name(err));
        return;
    }

    char id_str[8];

    snprintf(id_str, sizeof(id_str), "0x%02X", id);
    cJSON_AddStringToObject(o, "id", id_str);

    bool ok = (id == 0x67); /* ICM-42670-P WHO_AM_I */
    float ax = 0;
    float ay = 0;
    float az = 0;

    if (imu_manager_read_accel(&ax, &ay, &az) == ESP_OK)
    {
        cJSON_AddNumberToObject(o, "ax", round_to(ax, 1000.0f));
        cJSON_AddNumberToObject(o, "ay", round_to(ay, 1000.0f));
        cJSON_AddNumberToObject(o, "az", round_to(az, 1000.0f));
        cJSON_AddNumberToObject(o, "g",
                                round_to(sqrtf(ax * ax + ay * ay + az * az),
                                         1000.0f));
    }
    else
    {
        ok = false;
    }

    float temp_c = 0;

    if (imu_manager_read_temp(&temp_c) == ESP_OK)
    {
        cJSON_AddNumberToObject(o, "temp_c", round_to(temp_c, 10.0f));
    }

    int idle_low = -1;
    int idle_high = -1;

    err = imu_manager_int1_check(&idle_low, &idle_high);
    cJSON_AddStringToObject(o, "int1",
                            err == ESP_OK ? "ok"
                            : err == ESP_FAIL ? "fail" : esp_err_to_name(err));
    cJSON_AddNumberToObject(o, "int1_idle_active_low", idle_low);
    cJSON_AddNumberToObject(o, "int1_idle_active_high", idle_high);

    if (err != ESP_OK)
    {
        ok = false;
    }

    cJSON_AddBoolToObject(o, "ok", ok);
}

static void add_led(cJSON *root)
{
    static const char *MODE[] = { "off", "solid", "blink_slow", "blink_fast" };
    cJSON *o = cJSON_AddObjectToObject(root, "led");
    led_manager_prio_t prio;
    led_manager_state_t st;
    esp_err_t err = led_manager_active(&prio, &st);

    /* the arbiter's view only; the chip itself answers `led -i` / `led -d` */
    cJSON_AddBoolToObject(o, "ok", err == ESP_OK);

    if (err == ESP_OK && (size_t)st.mode < sizeof(MODE) / sizeof(MODE[0]))
    {
        cJSON_AddStringToObject(o, "mode", MODE[st.mode]);
        cJSON_AddNumberToObject(o, "r", st.r);
        cJSON_AddNumberToObject(o, "g", st.g);
        cJSON_AddNumberToObject(o, "b", st.b);
    }
    else if (err != ESP_OK)
    {
        cJSON_AddStringToObject(o, "err", esp_err_to_name(err));
    }
}

static void add_gps(cJSON *root)
{
    cJSON *o = cJSON_AddObjectToObject(root, "gps");
    bool rx = gps_is_functional(FACTORY_GPS_RX_WAIT_MS);

    cJSON_AddBoolToObject(o, "uart_rx", rx);

    /* the module answers only when the ESP to module direction works too */
    char ver[128] = "";
    esp_err_t err = rx ? gps_query_version(ver, sizeof(ver),
                                           FACTORY_GPS_VER_WAIT_MS)
                       : ESP_ERR_INVALID_STATE;

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(o, "version", ver);
    }
    else
    {
        cJSON_AddNullToObject(o, "version");
        cJSON_AddStringToObject(o, "version_err", esp_err_to_name(err));
    }

    gps_fix_snapshot_t fix;

    if (gps_get_fix(&fix) == ESP_OK)
    {
        cJSON_AddNumberToObject(o, "sats_in_view", fix.sats_in_view);
        cJSON_AddNumberToObject(o, "satellites", fix.satellites);
        cJSON_AddNumberToObject(o, "fix_quality", fix.fix_quality);
        cJSON_AddBoolToObject(o, "valid", fix.valid);
    }

    cJSON_AddBoolToObject(o, "ok", rx && err == ESP_OK);
}

static void add_storage(cJSON *root)
{
    cJSON *o = cJSON_AddObjectToObject(root, "storage");
    size_t total = 0;
    size_t used = 0;
    esp_err_t err = filesystem_info("/data", &total, &used);

    cJSON_AddBoolToObject(o, "mounted", err == ESP_OK);

    if (err == ESP_OK)
    {
        cJSON_AddNumberToObject(o, "total_kb", (double)(total / 1024));
        cJSON_AddNumberToObject(o, "used_kb", (double)(used / 1024));
    }
}

static void add_lte(cJSON *root)
{
    cJSON *o = cJSON_AddObjectToObject(root, "lte");
    bool enabled = lte_upstream_pppos_settings_is_configured() &&
                   lte_upstream_pppos_settings()->enabled;

    cJSON_AddBoolToObject(o, "enabled", enabled);

    if (enabled)
    {
        /* BG95 STATUS is low while the module is on (modem_manager owns
         * the pin; it is an input once the LTE stack has started) */
        cJSON_AddBoolToObject(o, "status_pin_on",
                              gpio_get_level(LTE_STATUS_PIN) == 0);
    }

    cJSON_AddStringToObject(o, "stage", lte_upstream_pppos_stage_str());
    cJSON_AddStringToObject(o, "sim", lte_upstream_pppos_sim_str(
                                          lte_upstream_pppos_sim_state()));
}

static void add_wifi(cJSON *root, const char *scan_ssid)
{
    cJSON *w = cJSON_AddObjectToObject(root, "wifi");

    cJSON_AddBoolToObject(w, "ap_started", wifi_manager_is_ap_started());
    cJSON_AddNumberToObject(w, "ap_clients",
                            wifi_manager_get_ap_station_count());

    if (scan_ssid == NULL)
    {
        return;
    }

    /* the fixture's AP at a fixed distance: the DUT-side RF check (about
     * 2 s; AP-only mode flips to APSTA for the scan and back) */
    cJSON *scan = cJSON_AddObjectToObject(w, "scan");

    cJSON_AddStringToObject(scan, "ssid", scan_ssid);

    char *json = wifi_manager_scan_networks();

    if (json == NULL)
    {
        cJSON_AddBoolToObject(scan, "ok", false);
        return;
    }

    cJSON *parsed = cJSON_Parse(json);

    free(json);

    const cJSON *nets = cJSON_GetObjectItemCaseSensitive(parsed, "networks");
    const cJSON *n = NULL;
    int count = 0;
    int best_rssi = -127;
    int best_channel = 0;
    bool found = false;

    cJSON_ArrayForEach(n, nets)
    {
        const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(n, "ssid");
        const cJSON *rssi = cJSON_GetObjectItemCaseSensitive(n, "rssi");
        const cJSON *channel = cJSON_GetObjectItemCaseSensitive(n, "channel");

        count++;

        if (cJSON_IsString(ssid) && cJSON_IsNumber(rssi) &&
            strcmp(ssid->valuestring, scan_ssid) == 0 &&
            (!found || rssi->valueint > best_rssi))
        {
            found = true;
            best_rssi = rssi->valueint;
            best_channel = cJSON_IsNumber(channel) ? channel->valueint : 0;
        }
    }

    cJSON_Delete(parsed);

    cJSON_AddBoolToObject(scan, "ok", true);
    cJSON_AddNumberToObject(scan, "count", count);
    cJSON_AddBoolToObject(scan, "found", found);

    if (found)
    {
        cJSON_AddNumberToObject(scan, "rssi", best_rssi);
        cJSON_AddNumberToObject(scan, "channel", best_channel);
    }
}

static int cmd_selftest(int argc, char **argv)
{
    const char *scan_ssid = NULL;

    for (int i = 2; i < argc; i++)
    {
        if (strcmp(argv[i], "-w") == 0 && i + 1 < argc)
        {
            scan_ssid = argv[++i];
        }
        else
        {
            printf("Usage: factory selftest [-w <ssid>]\n");
            return 1;
        }
    }

    int64_t t0 = now_ms();
    cJSON *root = cJSON_CreateObject();

    if (root == NULL)
    {
        printf("{\"error\":\"out of memory\"}\n");
        return 1;
    }

    add_identity(root);
    add_pins(root);
    add_imu(root);
    add_led(root);
    add_gps(root);
    add_storage(root);
    add_lte(root);
    add_wifi(root, scan_ssid);
    cJSON_AddNumberToObject(root, "ms", (double)(now_ms() - t0));

    print_json(root);
    cJSON_Delete(root);
    return 0;
}

/* ---- button ---------------------------------------------------------------- */

static int cmd_button(int argc, char **argv)
{
    long wait_ms = (argc >= 3) ? strtol(argv[2], NULL, 10) : 10000;

    if (wait_ms < FACTORY_BUTTON_MIN_MS || wait_ms > FACTORY_BUTTON_MAX_MS)
    {
        printf("Usage: factory button <ms> (%d..%d)\n", FACTORY_BUTTON_MIN_MS,
               FACTORY_BUTTON_MAX_MS);
        return 1;
    }

    /* main configured PB1 at boot; re-assert in case a sleep path touched it */
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    const int64_t deadline = now_ms() + wait_ms;
    int64_t pressed_at = 0;
    int64_t released_at = 0;
    int stable_ms = 0;
    bool pressed = false;
    bool released = false;

    while (now_ms() < deadline)
    {
        int level = gpio_get_level(BUTTON_GPIO); /* 0 = pressed (pull-up) */
        int want = pressed ? 1 : 0;

        if (level == want)
        {
            stable_ms += FACTORY_BUTTON_POLL_MS;

            if (stable_ms >= FACTORY_BUTTON_DEBOUNCE_MS)
            {
                if (!pressed)
                {
                    pressed = true;
                    pressed_at = now_ms();
                }
                else
                {
                    released = true;
                    released_at = now_ms();
                    break;
                }

                stable_ms = 0;
            }
        }
        else
        {
            stable_ms = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(FACTORY_BUTTON_POLL_MS));
    }

    printf("{\"pressed\":%s,\"released\":%s,\"hold_ms\":%ld,\"level\":%d}\n",
           pressed ? "true" : "false", released ? "true" : "false",
           released ? (long)(released_at - pressed_at) : 0L,
           gpio_get_level(BUTTON_GPIO));
    return (pressed && released) ? 0 : 1;
}

/* ---- USB mux ---------------------------------------------------------------- */

typedef struct
{
    int sel1;
    int sel2;
    uint32_t hold_ms;
} mux_job_t;

/* SEL1/SEL2 truth table (PI3USB102 pair; the schematic's silkscreen table
 * has the two switches swapped, the firmware boot gates are the truth):
 *   1/0 connector <-> ESP (the run position, NCM + console)
 *   0/0 connector <-> CH343 UART0 bridge (flash/log), ESP USB <-> BG95
 *   0/1 connector <-> BG95 USB passthrough
 *   1/1 everything disconnected (power only) */
static const struct
{
    const char *name;
    int sel1;
    int sel2;
} MUX_ROUTES[] =
{
    { "esp",   1, 0 },
    { "ch343", 0, 0 },
    { "bg95",  0, 1 },
    { "off",   1, 1 },
};

/* The deep-sleep path holds these pins and IDF v6's gpio_reset_pin does
 * not release a hold; INPUT_OUTPUT keeps the readback honest (the DIP is a
 * weak pull). Same drive as wifi_modem's runtime cut. */
static void mux_drive(int sel1, int sel2)
{
    (void)gpio_hold_dis(USB_SEL_1);
    (void)gpio_hold_dis(USB_SEL_2);
    gpio_set_direction(USB_SEL_1, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(USB_SEL_2, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(USB_SEL_1, sel1);
    gpio_set_level(USB_SEL_2, sel2);
}

/* Release the drive: the DIP switch pull selects the route again and the
 * host re-enumerates whatever sits on it. */
static void mux_release(void)
{
    (void)gpio_hold_dis(USB_SEL_1);
    (void)gpio_hold_dis(USB_SEL_2);
    gpio_set_direction(USB_SEL_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(USB_SEL_1, GPIO_FLOATING);
    gpio_set_direction(USB_SEL_2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(USB_SEL_2, GPIO_FLOATING);
}

static void usbmux_task(void *arg)
{
    mux_job_t job = *(mux_job_t *)arg;

    free(arg);

    vTaskDelay(pdMS_TO_TICKS(FACTORY_MUX_ANSWER_MS));

    /* a clean detach between routes: everything off first */
    mux_drive(1, 1);
    vTaskDelay(pdMS_TO_TICKS(FACTORY_MUX_DETACH_MS));
    mux_drive(job.sel1, job.sel2);
    vTaskDelay(pdMS_TO_TICKS(job.hold_ms));
    mux_drive(1, 1);
    vTaskDelay(pdMS_TO_TICKS(FACTORY_MUX_DETACH_MS));
    mux_release();
    vTaskDelete(NULL);
}

static int cmd_usbmux(int argc, char **argv)
{
    const char *route = (argc >= 3) ? argv[2] : "";
    long hold_ms = (argc >= 4) ? strtol(argv[3], NULL, 10) : 0;
    int idx = -1;

    for (size_t i = 0; i < sizeof(MUX_ROUTES) / sizeof(MUX_ROUTES[0]); i++)
    {
        if (strcmp(route, MUX_ROUTES[i].name) == 0)
        {
            idx = (int)i;
        }
    }

    if (idx < 0 || hold_ms < FACTORY_MUX_MIN_MS || hold_ms > FACTORY_MUX_MAX_MS)
    {
        printf("Usage: factory usbmux esp|ch343|bg95|off <ms> (%d..%d)\n",
               FACTORY_MUX_MIN_MS, FACTORY_MUX_MAX_MS);
        return 1;
    }

    mux_job_t *job = calloc(1, sizeof(*job));

    if (job == NULL)
    {
        printf("{\"error\":\"out of memory\"}\n");
        return 1;
    }

    job->sel1 = MUX_ROUTES[idx].sel1;
    job->sel2 = MUX_ROUTES[idx].sel2;
    job->hold_ms = (uint32_t)hold_ms;

    /* transient, respawnable: a dynamic internal stack is fine here (§2) */
    if (xTaskCreate(usbmux_task, "factory_mux", 3072, job, 5, NULL) != pdPASS)
    {
        free(job);
        printf("{\"error\":\"task\"}\n");
        return 1;
    }

    printf("{\"usbmux\":\"%s\",\"sel1\":%d,\"sel2\":%d,\"hold_ms\":%ld,"
           "\"applies_in_ms\":%d}\n",
           MUX_ROUTES[idx].name, MUX_ROUTES[idx].sel1, MUX_ROUTES[idx].sel2,
           hold_ms, FACTORY_MUX_ANSWER_MS);
    return 0;
}

/* ---- red LED (GPIO41, active-low; the LTE link indicator on v1.60) --------- */

static int cmd_led(int argc, char **argv)
{
    if (argc < 4 || strcmp(argv[2], "red") != 0)
    {
        printf("Usage: factory led red 0|1\n");
        return 1;
    }

    bool on = (atoi(argv[3]) != 0);

    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_RED_PIN, on ? 0 : 1);
    printf("{\"led_red\":%d}\n", on ? 1 : 0);
    return 0;
}

/* ---- factory record ------------------------------------------------------- */

static int mark_print(void)
{
    char *buf = malloc(FACTORY_RECORD_MAX + 1);

    if (buf == NULL)
    {
        printf("{\"error\":\"out of memory\"}\n");
        return 1;
    }

    size_t len = 0;
    esp_err_t err = filesystem_read(FACTORY_RECORD_PATH, buf,
                                    FACTORY_RECORD_MAX, &len);

    if (err != ESP_OK)
    {
        printf("{\"marked\":false,\"err\":\"%s\"}\n", esp_err_to_name(err));
        free(buf);
        return 0;
    }

    buf[len] = '\0';

    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
    {
        buf[--len] = '\0';
    }

    printf("%s\n", buf);
    free(buf);
    return 0;
}

static int cmd_mark(int argc, char **argv)
{
    if (argc < 3)
    {
        return mark_print();
    }

    const char *serial = argv[2];
    const char *station = (argc >= 4) ? argv[3] : "";

    if (serial[0] == '\0' || strlen(serial) > FACTORY_FIELD_MAX ||
        strlen(station) > FACTORY_FIELD_MAX)
    {
        printf("Usage: factory mark <serial> [station] (max %d chars each)\n",
               FACTORY_FIELD_MAX);
        return 1;
    }

    uint8_t mac[6] = { 0 };
    char mac_str[18];

    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    time_t now = time(NULL);
    cJSON *root = cJSON_CreateObject();

    if (root == NULL)
    {
        printf("{\"error\":\"out of memory\"}\n");
        return 1;
    }

    cJSON_AddBoolToObject(root, "marked", true);
    cJSON_AddStringToObject(root, "serial", serial);
    cJSON_AddStringToObject(root, "station", station);
    cJSON_AddStringToObject(root, "fw", dev_status_manager_app_version());
    cJSON_AddStringToObject(root, "device_id", dev_status_manager_device_id());
    cJSON_AddStringToObject(root, "mac", mac_str);
    /* unix time when the clock is set (LTE network time), else 0: the
     * host logs its own timestamp anyway */
    cJSON_AddNumberToObject(root, "ts",
                            (double)(now > 1600000000 ? (long long)now : 0));
    cJSON_AddNumberToObject(root, "uptime_s", (double)(now_ms() / 1000));

    char *s = cJSON_PrintUnformatted(root);

    cJSON_Delete(root);

    if (s == NULL)
    {
        printf("{\"error\":\"out of memory\"}\n");
        return 1;
    }

    /* atomic replace (temp + fsync + rename); the console task's stack is
     * internal RAM, so the flash write is legal here (§2 corollary) */
    esp_err_t err = filesystem_write(FACTORY_RECORD_PATH, s, strlen(s));

    if (err != ESP_OK)
    {
        printf("{\"marked\":false,\"err\":\"%s\"}\n", esp_err_to_name(err));
        cJSON_free(s);
        return 1;
    }

    printf("%s\n", s);
    cJSON_free(s);
    return 0;
}

/* ---- stored panic summary (bench diagnosis) ------------------------------- */

static int cmd_coredump(int argc, char **argv)
{
    if (argc >= 3 && strcmp(argv[2], "erase") == 0)
    {
        esp_err_t err = esp_core_dump_image_erase();

        printf("{\"erased\":%s}\n", err == ESP_OK ? "true" : "false");
        return err == ESP_OK ? 0 : 1;
    }

    esp_err_t err = esp_core_dump_image_check();

    if (err != ESP_OK)
    {
        printf("{\"coredump\":false,\"err\":\"%s\"}\n", esp_err_to_name(err));
        return 0;
    }

    /* the summary parses the ELF dump in flash; keep it off the console
     * task's stack */
    static esp_core_dump_summary_t s_summary EXT_RAM_BSS_ATTR;

    err = esp_core_dump_get_summary(&s_summary);

    if (err != ESP_OK)
    {
        printf("{\"coredump\":true,\"err\":\"%s\"}\n", esp_err_to_name(err));
        return 1;
    }

    cJSON *root = cJSON_CreateObject();

    if (root == NULL)
    {
        printf("{\"error\":\"out of memory\"}\n");
        return 1;
    }

    char hex[12];

    cJSON_AddBoolToObject(root, "coredump", true);
    cJSON_AddStringToObject(root, "task", s_summary.exc_task);
    snprintf(hex, sizeof(hex), "0x%08x", (unsigned)s_summary.exc_pc);
    cJSON_AddStringToObject(root, "pc", hex);
    cJSON_AddNumberToObject(root, "exc_cause", s_summary.ex_info.exc_cause);
    snprintf(hex, sizeof(hex), "0x%08x", (unsigned)s_summary.ex_info.exc_vaddr);
    cJSON_AddStringToObject(root, "exc_vaddr", hex);
    cJSON_AddStringToObject(root, "app_sha256",
                            (const char *)s_summary.app_elf_sha256);
    cJSON_AddBoolToObject(root, "bt_corrupted", s_summary.exc_bt_info.corrupted);

    cJSON *bt = cJSON_AddArrayToObject(root, "bt");

    for (uint32_t i = 0; i < s_summary.exc_bt_info.depth && i < 16; i++)
    {
        snprintf(hex, sizeof(hex), "0x%08x",
                 (unsigned)s_summary.exc_bt_info.bt[i]);
        cJSON_AddItemToArray(bt, cJSON_CreateString(hex));
    }

    print_json(root);
    cJSON_Delete(root);
    return 0;
}

/* ---- GPIO configuration dump (bench diagnosis) ----------------------------- */

static int cmd_gpio(int argc, char **argv)
{
    long pin = (argc >= 3) ? strtol(argv[2], NULL, 10) : -1;

    if (argc < 3 || pin < 0 || pin >= GPIO_NUM_MAX)
    {
        printf("Usage: factory gpio <0..%d>\n", GPIO_NUM_MAX - 1);
        return 1;
    }

    /* the driver's own report (function, direction, pulls, drive, hold,
     * sleep config) goes to stdout, which is this console */
    esp_err_t err = gpio_dump_io_configuration(stdout, 1ULL << pin);

    if (err != ESP_OK)
    {
        printf("gpio dump: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("level=%d\n", gpio_get_level((gpio_num_t)pin));
    return 0;
}

/* ---- reboot into ROM download mode ----------------------------------------- */

static int cmd_bootloader(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("{\"bootloader\":true,\"usb\":\"303A:0009\"}\n");
    vTaskDelay(pdMS_TO_TICKS(300)); /* the answer reaches the host first */

    /* modem back to AT mode first, like `reboot` */
    (void)lte_upstream_pppos_stop();

    /* The OTG stack routed the internal USB PHY to itself (RTC_CNTL
     * usb_conf), and that routing survives a soft reset, so the ROM's
     * download mode comes up on the OTG controller as 303A:0009 (CDC +
     * DFU; esptool flashes through it with --before no-reset). Handing the
     * PHY back to the serial-JTAG here was tried on 2026-10-15 and left the
     * chip with no USB at all until a power cycle; the OTG path is the
     * proven one, so the PHY is left alone. */

    /* keep the controller connected through the reset: the ROM's OTG
     * download device reuses it, and the usual restart detach would leave
     * the soft-disconnect bit set across the soft reset */
    usb_dev_ethernet_set_restart_detach(false);

    /* the ROM honours this flag on the next reset and stays in download
     * mode (the way IDF's USB console reboots to the bootloader) */
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    restart_tracker_restart(RESTART_TRACKER_PLANNED_REASON_USER_REQUEST,
                            RESTART_TRACKER_SOURCE_CONSOLE, 0);
    return 0; /* not reached */
}

/* ---- dispatcher ------------------------------------------------------------ */

static void print_usage(void)
{
    printf("Usage:\n");
    printf("  factory selftest [-w <ssid>]            hardware self-test as one JSON line\n");
    printf("  factory button <ms>                     wait for a PB1 press and release\n");
    printf("  factory usbmux esp|ch343|bg95|off <ms>  route the USB connector for <ms>\n");
    printf("  factory led red 0|1                     GPIO41 red LED\n");
    printf("  factory mark [<serial> [station]]       write (or print) /data/factory.json\n");
    printf("  factory coredump [erase]                the stored panic summary (or erase it)\n");
    printf("  factory gpio <n>                        dump one pin's live configuration\n");
    printf("  factory bootloader                      reboot into ROM download mode (303A:0009)\n");
}

static int cmd_factory(int argc, char **argv)
{
    if (argc < 2)
    {
        print_usage();
        return 1;
    }

    if (strcmp(argv[1], "selftest") == 0)
    {
        return cmd_selftest(argc, argv);
    }

    if (strcmp(argv[1], "button") == 0)
    {
        return cmd_button(argc, argv);
    }

    if (strcmp(argv[1], "usbmux") == 0)
    {
        return cmd_usbmux(argc, argv);
    }

    if (strcmp(argv[1], "led") == 0)
    {
        return cmd_led(argc, argv);
    }

    if (strcmp(argv[1], "mark") == 0)
    {
        return cmd_mark(argc, argv);
    }

    if (strcmp(argv[1], "coredump") == 0)
    {
        return cmd_coredump(argc, argv);
    }

    if (strcmp(argv[1], "gpio") == 0)
    {
        return cmd_gpio(argc, argv);
    }

    if (strcmp(argv[1], "bootloader") == 0)
    {
        return cmd_bootloader(argc, argv);
    }

    print_usage();
    return 1;
}

esp_err_t main_cli_register_factory(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "factory",
        .help    = "Production test: selftest [-w ssid] | button <ms> | "
                   "usbmux esp|ch343|bg95|off <ms> | led red 0|1 | "
                   "mark [<serial> [station]] | coredump [erase] | gpio <n> | "
                   "bootloader",
        .hint    = NULL,
        .func    = &cmd_factory,
    };

    return cmdline_manager_register(&CMD);
}

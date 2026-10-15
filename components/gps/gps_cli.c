/**
 * @file gps_cli.c
 * @brief The `gps` console command (§6b): registered via cmdline_manager on
 *        the "gps" settings boot apply, gated by the `cli` setting. Moved
 *        verbatim from gps.c (§1 file-size split) — outputs byte-identical.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "cmdline_manager.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gps.h"
#include "gps_internal.h"
#include "agnss_internal.h"

static bool s_cmd_registered = false;

static void gps_print_usage(void)
{
    printf("Usage: gps -p [-j] | gps -s | gps -t [0|1] | gps -U [0|1] | gps -c | gps -P [0|1] | gps -V\n");
}

// Weak symbol implemented by usb_cli_console so GPS component can switch input mode.
__attribute__((weak)) void usb_cli_console_set_gps_stream_mode(bool enabled)
{
    (void)enabled;
}

static void gps_print_last_fix(bool json)
{
    gps_fix_snapshot_t snap = {0};

    (void)gps_get_fix(&snap);

    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    bool agnss_enabled = false;
    (void)gps_settings_pending_bool("agnss_enabled", &agnss_enabled);

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
               "\"satellites\":%d,\"sats_in_view\":%d,"
               "\"fix_quality\":%d,\"fix_type\":%d,"
               "\"altitude_m\":%.1f,"
               "\"hdop\":%.1f,\"pdop\":%.1f,\"vdop\":%.1f,"
               "\"speed_knots\":%.2f,\"speed_kmph\":%.2f,"
               "\"course_deg\":%.2f,\"age_ms\":%u,"
               "\"agnss_enabled\":%s,\"time_injected\":%s,"
               "\"position_injected\":%s",
               snap.valid ? "true" : "false",
               snap.lat, snap.lon,
               snap.satellites, snap.sats_in_view,
               snap.fix_quality, snap.fix_type,
               snap.altitude_m,
               snap.hdop, snap.pdop, snap.vdop,
               snap.speed_knots, snap.speed_kmph,
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
            "valid=%d lat=%.7f lon=%.7f sat=%d/%d fixq=%d type=%d alt=%.1fm "
            "hdop=%.1f pdop=%.1f vdop=%.1f spd=%.2fkn/%.1fkm/h crs=%.2fdeg age=%ums\n",
            snap.valid,
            snap.lat, snap.lon,
            snap.satellites, snap.sats_in_view,
            snap.fix_quality, snap.fix_type,
            snap.altitude_m,
            snap.hdop, snap.pdop, snap.vdop,
            snap.speed_knots, snap.speed_kmph,
            snap.course_deg,
            (unsigned)age_ms);
    }
}

static int cmd_gps(int argc, char **argv)
{
    bool opt_stream = false;
    bool opt_print = false;
    bool opt_json = false;
    bool opt_passthru = false;
    bool opt_uart0 = false;
    bool opt_check = false;
    bool opt_power = false;
    bool opt_version = false;

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
        else if (strcmp(argv[i], "-t") == 0)
        {
            opt_passthru = true;
            if (i + 1 < argc && argv[i + 1][0] != '-')
            {
                i++;
            }
        }
        else if (strcmp(argv[i], "-U") == 0)
        {
            opt_uart0 = true;
            if (i + 1 < argc && argv[i + 1][0] != '-')
            {
                i++;
            }
        }
        else if (strcmp(argv[i], "-c") == 0)
        {
            opt_check = true;
        }
        else if (strcmp(argv[i], "-P") == 0)
        {
            opt_power = true;
            if (i + 1 < argc && argv[i + 1][0] != '-')
            {
                i++;
            }
        }
        else if (strcmp(argv[i], "-V") == 0)
        {
            opt_version = true;
        }
        else
        {
            gps_print_usage();
            return 1;
        }
    }

    if (opt_version)
    {
        /* the module's answer proves both UART directions and names the
         * GNSS firmware (the production test records it) */
        (void)gps_init(false);

        char ver[128];
        esp_err_t err = gps_query_version(ver, sizeof(ver), 3000);

        if (err != ESP_OK)
        {
            printf("GPS version: %s\n", esp_err_to_name(err));
            return 1;
        }

        printf("GPS version: %s\n", ver);
        return 0;
    }

    if (opt_power)
    {
        int idx = -1;
        for (int i = 1; i < argc; i++)
        {
            if (strcmp(argv[i], "-P") == 0 && i + 1 < argc)
            {
                idx = i + 1;
                break;
            }
        }

        if (idx < 0)
        {
            printf("GPS power: %s\n", gps_power_pin_is_enabled() ? "on" : "off");
            return 0;
        }

        int val = atoi(argv[idx]);
        if (val != 0 && val != 1)
        {
            printf("Usage: gps -P [0|1]\n");
            return 1;
        }

        if (val != 0)
        {
            esp_err_t err = gps_init(false);
            if (err != ESP_OK)
            {
                printf("GPS power on failed: %s\n", esp_err_to_name(err));
                return 1;
            }
        }
        else
        {
            gps_set_streaming(false);
            usb_cli_console_set_gps_stream_mode(false);
        }

        esp_err_t err = gps_power_pin_set(val != 0);
        if (err != ESP_OK)
        {
            printf("GPS power change failed: %s\n", esp_err_to_name(err));
            return 1;
        }

        printf("GPS power: %s\n", val != 0 ? "on" : "off");
        return 0;
    }

    if (opt_uart0)
    {
        int idx = -1;
        for (int i = 1; i < argc; i++)
        {
            if (strcmp(argv[i], "-U") == 0 && i + 1 < argc)
            {
                idx = i + 1;
                break;
            }
        }

        if (idx < 0)
        {
            printf("GPS UART0 routing: %s\n",
                   gps_is_uart0_output_enabled() ? "enabled" : "disabled");
            return 0;
        }

        int val = atoi(argv[idx]);
        if (val != 0 && val != 1)
        {
            printf("Usage: gps -U [0|1]\n");
            return 1;
        }

        bool enable = (val != 0);
        esp_err_t err = gps_settings_persist_bool("uart0_out", enable);
        if (err != ESP_OK)
        {
            printf("GPS UART0 routing update failed: %s\n", esp_err_to_name(err));
            return 1;
        }

        if (enable)
        {
            err = gps_init(false);
            if (err != ESP_OK)
            {
                printf("GPS UART0 routing enable failed: %s\n", esp_err_to_name(err));
                return 1;
            }

            err = gps_uart0_prepare();
            if (err != ESP_OK)
            {
                printf("GPS UART0 prepare failed: %s\n", esp_err_to_name(err));
                return 1;
            }
        }

        gps_set_uart0_output(enable);
        printf("GPS UART0 routing %s (persistent).\n",
               enable ? "enabled" : "disabled");
        return 0;
    }

    if (opt_check)
    {
        (void)gps_init(false);
        bool ok = gps_is_functional(2000);
        printf("GPS UART: %s\n", ok ? "receiving data" : "no data");
        return ok ? 0 : 1;
    }

    if (opt_passthru)
    {
        /* gps -t       → show current state
         * gps -t 0|1   → set and persist */
        int idx = -1;
        for (int i = 1; i < argc; i++)
        {
            if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
            {
                idx = i + 1;
                break;
            }
        }
        if (idx < 0)
        {
            bool cur = false;
            (void)gps_settings_pending_bool("passthru", &cur);
            printf("GPS passthrough: %s\n", cur ? "enabled" : "disabled");
            return 0;
        }
        int val = atoi(argv[idx]);
        bool enable = (val != 0);
        (void)gps_settings_persist_bool("passthru", enable);
        if (enable)
        {
            (void)gps_init(false);
            gps_set_streaming(true);
            usb_cli_console_set_gps_stream_mode(true);
            printf("GPS passthrough enabled (persistent). Ctrl+C to exit.\n");
        }
        else
        {
            gps_set_streaming(false);
            printf("GPS passthrough disabled.\n");
        }
        return 0;
    }

    if (opt_stream)
    {
        (void)gps_init(false);
        gps_set_streaming(true);
        usb_cli_console_set_gps_stream_mode(true);
        printf("GPS stream mode. Ctrl+C or +++ Enter to exit.\n");
        return 0;
    }

    if (!opt_print)
    {
        gps_print_usage();
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
        .help = "GPS: -p [-j] print fix, -s stream NMEA, -t [0|1] USB passthrough, -U [0|1] route NMEA to UART0, -c check UART, -P [0|1] power, -V module firmware version",
        .hint = NULL,
        .func = &cmd_gps,
    };

    (void)cmdline_manager_register(&cmd);
    s_cmd_registered = true;
}

/**
 * @file wifi_modem_cli.c
 * @brief The `wifi_modem` console command (§6b): registered into
 *        cmdline_manager on the "wifi_modem" settings boot apply, gated by
 *        the `cli` setting.
 *
 *   wifi_modem                     status summary (one line)
 *   wifi_modem -k                  AP credentials (SSID + password) — the
 *                                  only place the per-device password is
 *                                  ever printed; console surfaces need
 *                                  physical/USB access
 *   wifi_modem -u auto|ncm|power_only  persist usb_mode (reboot to apply)
 *   wifi_modem -a 0|1              persist ap_share (reboot to apply)
 *   wifi_modem -d 0|1              runtime USB data cut / restore (what the
 *                                  WiCAN does over HTTP; bench + diagnosis)
 */
#include "wifi_modem.h"

#include <stdlib.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_wifi.h"

#include "cmdline_manager.h"
#include "hw_config.h"
#include "wifi_manager.h"

#include "wifi_modem_private.h"

static struct
{
    struct arg_lit *key;
    struct arg_str *usb;
    struct arg_int *share;
    struct arg_int *data;
    struct arg_end *end;
} s_args;

static void print_status(void)
{
    const wifi_modem_settings_t *cfg = wifi_modem_settings_config();
    char ap_ip[16] = "-";
    char dns[16] = "-";

    (void)wifi_manager_get_ap_ip(ap_ip, sizeof(ap_ip));
    if (wifi_modem_upstream_dns() != 0)
    {
        (void)wmc_format_ipv4(wifi_modem_upstream_dns(), dns, sizeof(dns));
    }

    cmdline_printf("usb_mode=%s usb_data=%d ap_share=%d napt=%d ap_ip=%s "
                   "clients=%u upstream_dns=%s mux SEL1=%d SEL2=%d\n",
                   wmc_usb_mode_str(cfg->usb_mode),
                   (int)wifi_modem_usb_data_routed(), (int)cfg->ap_share,
                   (int)wifi_modem_napt_active(), ap_ip,
                   (unsigned)wifi_manager_get_ap_station_count(), dns,
                   gpio_get_level(USB_SEL_1), gpio_get_level(USB_SEL_2));
}

static int print_key(void)
{
    wifi_config_t wc;

    if (esp_wifi_get_config(WIFI_IF_AP, &wc) != ESP_OK)
    {
        cmdline_printf("AP not configured\n");
        return 1;
    }

    cmdline_printf("ap_ssid=%s ap_password=%s\n", (const char *)wc.ap.ssid,
                   (const char *)wc.ap.password);
    return 0;
}

static int cmd_wifi_modem(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&s_args);

    if (nerrors != 0)
    {
        cmdline_printf("usage: wifi_modem [-k] [-u auto|ncm|power_only] "
                       "[-a 0|1] [-d 0|1]\n");
        return 1;
    }

    if (s_args.data->count > 0)
    {
        bool on = s_args.data->ival[0] != 0;
        esp_err_t err = wifi_modem_usb_data_set(on);

        cmdline_printf("usb_data=%d %s (SEL1=%d SEL2=%d)\n",
                       (int)wifi_modem_usb_data_routed(),
                       err == ESP_OK ? "ok" : esp_err_to_name(err),
                       gpio_get_level(USB_SEL_1), gpio_get_level(USB_SEL_2));
        return err == ESP_OK ? 0 : 1;
    }

    if (s_args.usb->count > 0)
    {
        wifi_modem_usb_mode_t mode;

        if (!wmc_usb_mode_parse(s_args.usb->sval[0], &mode))
        {
            cmdline_printf("usage: wifi_modem -u auto|ncm|power_only\n");
            return 1;
        }

        esp_err_t err = wifi_modem_settings_persist_str("usb_mode",
                                                        wmc_usb_mode_str(mode));

        cmdline_printf("usb_mode=%s %s (reboot to apply)\n",
                       wmc_usb_mode_str(mode),
                       err == ESP_OK ? "saved" : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }

    if (s_args.share->count > 0)
    {
        bool on = s_args.share->ival[0] != 0;
        esp_err_t err = wifi_modem_settings_persist_bool("ap_share", on);

        cmdline_printf("ap_share=%d %s (reboot to apply)\n", (int)on,
                       err == ESP_OK ? "saved" : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }

    if (s_args.key->count > 0)
    {
        return print_key();
    }

    print_status();
    return 0;
}

esp_err_t wifi_modem_register_cli(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "wifi_modem",
        .help = "WiFi-modem mode: status, AP credentials, usb_mode/ap_share "
                "(reboot to apply), runtime USB data cut",
        .hint = "Options: -k/--key, -u/--usb auto|ncm|power_only, "
                "-a/--share 0|1, -d/--data 0|1",
        .func = cmd_wifi_modem,
        .argtable = &s_args,
    };

    s_args.key = arg_lit0("k", "key", "Print the AP SSID and password");
    s_args.usb = arg_str0("u", "usb", "<mode>",
                          "Persist usb_mode: auto | ncm | power_only");
    s_args.share = arg_int0("a", "share", "<0|1>", "Persist ap_share");
    s_args.data = arg_int0("d", "data", "<0|1>",
                           "Runtime USB data lines: 0 = cut, 1 = restore");
    s_args.end = arg_end(4);
    return cmdline_manager_register(&CMD);
}

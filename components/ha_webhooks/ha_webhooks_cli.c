/**
 * @file ha_webhooks_cli.c
 * @brief The `webhook` console command — shows the HA link config + stats.
 *        Registered by ha_webhooks_register_cli() from on_apply, gated by
 *        the `cli` setting (§6b). main wires nothing.
 */
#include "cmdline_manager.h"

#include "ha_webhooks.h"
#include "ha_webhooks_private.h"

static int cmd_webhook(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    hw_config_t c = { 0 };
    hw_stats_t s;

    if (!hw_config_get(&c))
    {
        cmdline_printf("webhook: not configured\n");
        return 0;
    }

    hw_stats_get(&s);

    cmdline_printf("enabled:   %s\n", c.enabled ? "yes" : "no");
    cmdline_printf("url:       %s\n", c.url[0] ? c.url : "(none)");

    if (c.url2[0])
    {
        cmdline_printf("url2:      %s\n", c.url2);
    }

    cmdline_printf("interval:  %us\n", (unsigned)c.interval_s);
    cmdline_printf("data_mode: %s\n", c.data_mode_full ? "full" : "changed");
    cmdline_printf("override:  %s\n", c.manual_override ? "yes" : "no");
    cmdline_printf("cert_set:  %s\n", c.cert_set[0] ? c.cert_set : "(bundle)");
    cmdline_printf("status:    %s\n", s.status[0] ? s.status : "disabled");
    cmdline_printf("last_post: %s\n", s.last_post[0] ? s.last_post : "-");
    cmdline_printf("success:   %u  fail: %u  retries: %u\n",
                   (unsigned)s.success_count, (unsigned)s.fail_count,
                   (unsigned)s.retries);

    if (s.last_error[0])
    {
        cmdline_printf("last_err:  %s (%s)\n", s.last_error,
                       s.last_error_time);
    }

    return 0;
}

esp_err_t ha_webhooks_register_cli(void)
{
    static const esp_console_cmd_t CMD = {
        .command = "webhook",
        .help = "Home Assistant webhook link config + stats",
        .func = cmd_webhook,
    };

    return cmdline_manager_register(&CMD);
}

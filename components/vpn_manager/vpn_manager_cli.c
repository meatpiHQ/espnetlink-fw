/**
 * @file vpn_manager_cli.c
 * @brief The `vpn` CLI command — registered by
 *        vpn_manager_register_cli() on the settings apply (§6b).
 */
#include "cmdline_manager.h"

#include "vpn_manager.h"

static const char *state_str(vpn_state_t st)
{
    switch (st)
    {
        case VPN_STATE_WAITING:    return "waiting for network/time";
        case VPN_STATE_CONNECTING: return "connecting";
        case VPN_STATE_CONNECTED:  return "connected";
        default:                   return "disabled";
    }
}

static int cmd_vpn(int argc, char **argv)
{
    vpn_manager_status_t st;

    (void)argc;
    (void)argv;
    (void)vpn_manager_status(&st);

    cmdline_printf("vpn: %s, %s (%s)\n",
                   st.tailscale ? "tailscale" : "wireguard",
                   state_str(st.state),
                   st.endpoint[0] ? st.endpoint : "no endpoint");

    if (st.tailscale && st.ts_ip[0])
    {
        cmdline_printf("  tailnet ip %s, %d peer(s)\n", st.ts_ip,
                       st.ts_peers);
    }

    cmdline_printf("  connects %lu, failures %lu, uptime %lu s\n",
                   (unsigned long)st.connects,
                   (unsigned long)st.failures,
                   (unsigned long)st.uptime_s);
    return 0;
}

esp_err_t vpn_manager_register_cli(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "vpn",
        .help = "VPN (WireGuard) status",
        .func = cmd_vpn,
    };

    return cmdline_manager_register(&CMD);
}

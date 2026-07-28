idf_component_register(
    SRCS "vpn_manager_events.c" "vpn_manager.c"
         "vpn_manager_settings.c"
         "vpn_manager_wg.c"
         "vpn_manager_ts.c"
         "vpn_manager_check.c"
         "vpn_manager_keygen.c"
         "vpn_manager_http.c"
         "vpn_manager_cli.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Feature: THE VPN client — user-selected `type`: plain WireGuard
    # (vendored esp_wireguard API) or Tailscale (vendored microlink).
    # Both share ONE WireGuard core (wireguard_lwip). /api/vpn and the
    # `vpn` command are optional (main wires them).
    # espnetlink port: event_manager dropped (vpn_manager_events.c kept
    # on disk but excluded from SRCS; call sites are no-ops) and
    # rtc_manager dropped (no such component here — the clock gate is a
    # local time(NULL) sanity floor in vpn_manager.c).
    PRIV_REQUIRES esp_wireguard wireguard_lwip microlink esp_netif lwip mbedtls esp_hw_support
                  dev_status_manager
                  http_server_manager settings_manager log_manager
                  cmdline_manager
)

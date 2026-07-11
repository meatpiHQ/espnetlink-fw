idf_component_register(
    SRCS "src/esp_wireguard.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    # The trombik convenience API only. The WireGuard CORE (wireguardif,
    # crypto, platform) is the SHARED components/wireguard_lwip — ONE
    # data plane + ONE set of wireguardif_*/crypto symbols for BOTH VPN
    # types (plain WireGuard here, Tailscale via microlink). The private
    # esp_wireguard core copies were deleted 2026-07-07 when microlink
    # landed, to avoid duplicate symbols at link (PROVENANCE.md). Two
    # small esp_wireguard.c adaptations to microlink's core: renamed a
    # local var off the core's wireguard_peer_index() function, and
    # dropped the explicit wireguard_platform_init() (the shared
    # platform self-inits).
    REQUIRES esp_netif lwip
    PRIV_REQUIRES wireguard_lwip mbedtls)

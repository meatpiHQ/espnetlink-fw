set(SRCS
    "src/microlink.c"
    "src/ml_coord.c"
    "src/ml_derp.c"
    "src/ml_net_io.c"
    "src/ml_wg_mgr.c"
    "src/ml_stun.c"
    "src/ml_noise.c"
    "src/ml_h2.c"
    "src/ml_udp.c"
    "src/ml_tcp.c"
    "src/ml_peer_nvs.c"
    "src/nacl_box.c"
    "src/x25519.c"
    "src/ml_zerocopy.c"
    "src/ml_cellular.c"
    "src/ml_at_socket.c"
    "src/ml_net_switch.c"
)

set(REQUIRES
    nvs_flash
    esp_wifi
    esp_netif
    esp_event
    esp_timer
    lwip
    mbedtls
    # espnetlink/IDF v5.5: cJSON from the bundled `json` component
    # (upstream microlink's original dep; WiCAN/IDF v6 uses the
    # espressif__cjson managed component instead).
    json
    esp_driver_uart
    esp_driver_gpio
    esp_http_server
)

# espnetlink/IDF v5.5 (mbedtls 3.x): entropy/ctr_drbg/chacha20/chachapoly
# headers are public — the WiCAN/IDF v6 (mbedtls 4 / TF-PSA-Crypto)
# private-include-dir hack is dropped; ml_noise.c includes the classic
# mbedtls/ paths again (see PROVENANCE.md, ESPNetLink port section).

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS
        "include"
        "src"
    REQUIRES ${REQUIRES}
    PRIV_REQUIRES
        wireguard_lwip
)

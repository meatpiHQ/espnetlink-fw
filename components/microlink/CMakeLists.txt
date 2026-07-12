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
    # cJSON from the espressif/cjson managed component (IDF v6 dropped the
    # bundled `json` component; see idf_component.yml), matching WiCAN v6.
    espressif__cjson
    esp_driver_uart
    esp_driver_gpio
    esp_http_server
)

# IDF v6: mbedtls 4 (TF-PSA-Crypto) moved entropy/ctr_drbg/chacha20/
# chachapoly to mbedtls/private/. Add the builtin private include dir so
# microlink's classic-mbedtls TLS/crypto (byte-unchanged) resolves them
# (matches WiCAN v6; see PROVENANCE.md, ESPNetLink port section).
idf_build_get_property(idf_path IDF_PATH)
set(_ml_mbedtls_priv
    "${idf_path}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS
        "include"
        "src"
        "${_ml_mbedtls_priv}"
    REQUIRES ${REQUIRES}
    PRIV_REQUIRES
        wireguard_lwip
)

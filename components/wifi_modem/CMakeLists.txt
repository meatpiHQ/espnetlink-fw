idf_component_register(
    SRCS "wifi_modem.c"
         "wifi_modem_core.c"
         "wifi_modem_migrate.c"
         "wifi_modem_settings.c"
         "wifi_modem_provision.c"
         "wifi_modem_dns.c"
         "wifi_modem_http.c"
         "wifi_modem_cli.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Feature: WiFi-modem mode (USB-C = 5 V only; AP NAPT into the LTE
    # uplink; per-device AP password; /api/wifi_modem; `wifi_modem` CLI).
    # The public header exposes only esp_err_t/stdbool, so every dependency
    # is private (§3: favour PRIV_REQUIRES to keep the graph shallow).
    # cJSON = espressif/cjson managed component (IDF v6 dropped `json`).
    PRIV_REQUIRES settings_manager log_manager wifi_manager hw_config
                  cmdline_manager http_server_manager gps
                  lte_upstream_pppos dev_status_manager
                  esp_driver_gpio esp_netif esp_event lwip esp_wifi
                  esp_timer esp_hw_support esp_http_server console
                  espressif__cjson
)

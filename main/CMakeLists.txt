idf_component_register(
    SRCS "main.c"
         "main_settings.c"
         "main_cli.c"
         "main_cli_system.c"
         "main_cli_net.c"
         "main_legacy_import.c"
    INCLUDE_DIRS "."
    REQUIRES usb usb_cdc_host_manager modem_manager usb_dev_bridge usb_dev_ethernet hw_config lte_upstream_pppos filesystem nvs_flash gps wifi_modem
             i2c_bus imu_manager vpn_manager cert_manager usb_cli_console
             log_manager settings_manager cmdline_manager http_server_manager
             dev_status_manager wifi_manager api_http restart_tracker
             ota_manager ha_webhooks mdns_manager web_ui http_client_manager
             esp_netif esp_event
             console espressif__cjson esp_http_client esp-tls lwip esp_timer
             esp_driver_gpio esp_driver_uart espressif__iperf
             esp_hw_support esp_hal_usb
)


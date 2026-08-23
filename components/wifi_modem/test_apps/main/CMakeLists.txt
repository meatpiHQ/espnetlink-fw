idf_component_register(
    SRCS "test_wifi_modem_main.c"
    INCLUDE_DIRS "."
    REQUIRES wifi_modem wifi_manager settings_manager dev_status_manager
             log_manager cmdline_manager hw_config nvs_flash esp_driver_gpio
             esp_netif esp_event espressif__cjson
)

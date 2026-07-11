idf_component_register(
    SRCS "hil_main.c"
    REQUIRES wifi_manager settings_manager nvs_flash esp_driver_uart
)

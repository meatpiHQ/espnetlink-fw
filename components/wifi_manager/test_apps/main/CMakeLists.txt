idf_component_register(
    SRCS "test_wifi_main.c"
    REQUIRES wifi_manager settings_manager dev_status_manager nvs_flash
)

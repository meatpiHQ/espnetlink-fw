idf_component_register(
    SRCS "usb_cli_console.c"
         "usb_cli_commands.c"
    INCLUDE_DIRS "include"
    REQUIRES cherry-embedded__cherryusb freertos console vfs esp_common gps lte_upstream_pppos config_manager esp_http_client esp-tls lwip imu hw_config driver esp_hw_support
)

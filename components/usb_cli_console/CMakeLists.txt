idf_component_register(
    SRCS "usb_cli_console.c"
    INCLUDE_DIRS "include"
    REQUIRES cherry-embedded__cherryusb freertos console vfs esp_common gps lte_upstream_pppos config_manager
)

idf_component_register(
    SRCS "usb_cli_console.c"
         "usb_cli_console_cli.c"
         "usb_cli_console_settings.c"
         "usb_cli_ota.c"
    INCLUDE_DIRS "include"
    # gps: passthrough auto-start reads the gps settings + stream hooks.
    # settings_manager/cmdline_manager: the {cli} descriptor + own commands.
    REQUIRES cherryusb freertos console vfs esp_common gps
             hw_config driver esp_hw_support settings_manager cmdline_manager
             espressif__cjson
    PRIV_REQUIRES app_update mbedtls
)

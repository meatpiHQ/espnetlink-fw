idf_component_register(
    SRCS "cmdline_manager.c"
         "cmdline_manager_line.c"
         "cmdline_manager_console.c"
         "cmdline_manager_settings.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Service: ONE command registry (esp_console) + dispatcher behind the
    # standard chunk-endpoint face. It owns NO commands except `help` —
    # components register their own via cmdline_manager_register()
    # (<comp>_register_cli(), wired by main). console is PUBLIC: the
    # register API takes esp_console_cmd_t.
    REQUIRES console
    PRIV_REQUIRES settings_manager log_manager esp_driver_uart json
)

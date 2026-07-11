idf_component_register(
    SRCS "restart_tracker.c"
         "restart_tracker_core.c"
         "restart_tracker_cli.c"
         "restart_tracker_settings.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # cache msync + esp_timer are only used by the .c files;
    # cmdline_manager only by the _cli.c (`restart_tracker` command)
    PRIV_REQUIRES esp_mm esp_timer log_manager cmdline_manager
                  settings_manager
)

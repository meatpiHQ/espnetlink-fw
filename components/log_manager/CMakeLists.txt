idf_component_register(
    SRCS "log_manager.c"
         "log_manager_ring.c"
         "log_manager_settings.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # public header exposes esp_log_level_t -> log (esp_log) is public;
    # settings registration + cache msync are .c-only.
    REQUIRES log
    PRIV_REQUIRES settings_manager esp_mm
)

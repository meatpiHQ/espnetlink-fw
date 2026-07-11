idf_component_register(
    SRCS "modem_manager.c"
         "modem_manager_parse.c"
         "modem_manager_probe.c"
    INCLUDE_DIRS "include" "${PROJECT_DIR}/main"
    REQUIRES driver esp_timer
)
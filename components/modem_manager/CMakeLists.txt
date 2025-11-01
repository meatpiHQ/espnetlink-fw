idf_component_register(
    SRCS "modem_manager.c"
    INCLUDE_DIRS "include" "${PROJECT_DIR}/main"
    REQUIRES driver esp_timer
)
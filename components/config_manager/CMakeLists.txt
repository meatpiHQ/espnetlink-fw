idf_component_register(
    SRCS "config_manager_console.c" "config_manager.c" "config_schema.c"
    INCLUDE_DIRS "include"
    REQUIRES filesystem json freertos console
)

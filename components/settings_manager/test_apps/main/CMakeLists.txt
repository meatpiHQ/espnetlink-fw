# cJSON comes transitively from settings_manager (public dep via its manifest).
idf_component_register(
    SRCS "test_settings_main.c"
    REQUIRES settings_manager
)

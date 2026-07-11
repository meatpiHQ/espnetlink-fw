idf_component_register(
    SRCS "gps.c"
         "agnss.c"
         "agnss_nvs.c"
         "agnss_console.c"
         "gps_settings.c"
         "gps_cli.c"
    INCLUDE_DIRS "include"
    # settings_manager is public: the descriptor/getters live in the public
    # header surface (gps.h) and gps_settings.c uses cJSON (bundled json).
    REQUIRES driver freertos esp_timer console hw_config igrr__libnmea
             lte_upstream_pppos nvs_flash settings_manager cmdline_manager json
    WHOLE_ARCHIVE
)

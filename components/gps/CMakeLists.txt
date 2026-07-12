idf_component_register(
    SRCS "gps.c"
         "agnss.c"
         "agnss_nvs.c"
         "agnss_console.c"
         "gps_settings.c"
         "gps_cli.c"
         "gps_http.c"
    INCLUDE_DIRS "include"
    # settings_manager is public: the descriptor/getters live in the public
    # header surface (gps.h) and gps_settings.c uses cJSON (espressif/cjson).
    REQUIRES esp_driver_gpio esp_driver_uart freertos esp_timer console hw_config igrr__libnmea
             lte_upstream_pppos nvs_flash settings_manager cmdline_manager
             espressif__cjson
    PRIV_REQUIRES http_server_manager esp_http_server
    WHOLE_ARCHIVE
)

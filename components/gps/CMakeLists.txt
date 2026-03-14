idf_component_register(
    SRCS "gps.c"
         "agnss.c"
         "agnss_nvs.c"
         "agnss_console.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer console hw_config igrr__libnmea
             lte_upstream_pppos nvs_flash config_manager
    WHOLE_ARCHIVE
)

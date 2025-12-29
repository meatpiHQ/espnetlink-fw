idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer console hw_config igrr__libnmea
)

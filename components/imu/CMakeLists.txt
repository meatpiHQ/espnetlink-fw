idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c console hw_config espressif__icm42670
)

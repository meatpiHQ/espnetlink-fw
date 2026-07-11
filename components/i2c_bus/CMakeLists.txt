idf_component_register(
    SRCS "i2c_bus.c"
    INCLUDE_DIRS "include"
    # HAL: owns the one shared I2C master bus; peripheral managers attach
    # devices via i2c_bus_add_device (never create a bus themselves).
    # REQUIRES (public): the header exposes i2c_master handle types.
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES log_manager
)

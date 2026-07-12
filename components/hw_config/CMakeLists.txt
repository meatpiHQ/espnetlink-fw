idf_component_register(
    SRCS
    INCLUDE_DIRS "include"
    # hw_config.h includes driver/gpio.h + driver/uart.h; on IDF v6 these
    # live in the split esp_driver_* components (public so consumers get
    # the include dirs transitively).
    REQUIRES esp_driver_gpio esp_driver_uart
)

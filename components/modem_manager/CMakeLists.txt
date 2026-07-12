idf_component_register(
    SRCS "modem_manager.c"
         "modem_manager_parse.c"
         "modem_manager_probe.c"
    INCLUDE_DIRS "include" "${PROJECT_DIR}/main"
    # esp_driver_gpio/esp_driver_uart are public: modem_manager.h includes
    # driver/gpio.h + driver/uart.h (IDF v6 removed them from `driver`).
    REQUIRES esp_driver_gpio esp_driver_uart esp_timer
)
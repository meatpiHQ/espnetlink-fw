idf_component_register(
    SRCS
        "usb_dev_bridge.c"
        "usb_descriptors.c"
    INCLUDE_DIRS "include"
    REQUIRES espressif__esp_tinyusb console igrr__libnmea modem_manager hw_config
)

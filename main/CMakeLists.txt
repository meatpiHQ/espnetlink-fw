idf_component_register(
    SRCS "main.c" "usb_dev_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES  usb espressif__esp_tinyusb usb_cdc_host_manager modem_manager console igrr__libnmea
)

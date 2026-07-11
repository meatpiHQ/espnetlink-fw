idf_component_register(
    SRCS "test_usbnet_main.c"
    REQUIRES usb_dev_ethernet esp_netif esp_event
)

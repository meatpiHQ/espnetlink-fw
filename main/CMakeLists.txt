idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES  usb usb_cdc_host_manager modem_manager
)

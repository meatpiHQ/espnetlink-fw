idf_component_register(
    SRCS "usb_dev_ethernet_rndis.c"
         "usb_dev_ethernet_settings.c"
         "usbnet_class.c"
         "usbnet_state.c"
         "usbnet_usb.c"
         "usbnet_lwip.c"
         "usbnet_ncm.c"
         "usbnet_dns_proxy.c"
         "usbnet_dhcp_debug.c"
    INCLUDE_DIRS "include"
    REQUIRES cherryusb esp_netif lwip usb_cli_console settings_manager espressif__cjson
)

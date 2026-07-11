idf_component_register(
    SRCS "usb_dev_ethernet_rndis.c"
         "usbnet_state.c"
         "usbnet_usb.c"
         "usbnet_lwip.c"
         "usbnet_dns_proxy.c"
         "usbnet_dhcp_debug.c"
    INCLUDE_DIRS "include"
    REQUIRES cherry-embedded__cherryusb esp_netif lwip usb_cli_console
)

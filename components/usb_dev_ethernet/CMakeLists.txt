set(TINYUSB_NETLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../managed_components/espressif__tinyusb/lib/networking")

idf_component_register(
    SRCS "usb_dev_ethernet_rndis.c"
         "usbnet_state.c"
         "usbnet_usb.c"
         "usbnet_lwip.c"
         "usbnet_dns_proxy.c"
         "usbnet_dhcp_debug.c"
         "${TINYUSB_NETLIB_DIR}/dhserver.c"
    INCLUDE_DIRS "include"
                 "${TINYUSB_NETLIB_DIR}"
    REQUIRES cherry-embedded__cherryusb esp_netif lwip usb_cli_console
)

# ESP-IDF builds with warnings-as-errors; TinyUSB's dhserver.c can trigger -Waddress
# due to LWIP macros checking pointer non-nullness. Keep the upstream file intact
# but relax this warning for that translation unit only.
set_source_files_properties(
    "${TINYUSB_NETLIB_DIR}/dhserver.c"
    PROPERTIES COMPILE_FLAGS "-Wno-error=address -Wno-address"
)

set(TINYUSB_NETLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../managed_components/espressif__tinyusb/lib/networking")

idf_component_register(
    SRCS "usb_dev_ethernet.c"
         "${TINYUSB_NETLIB_DIR}/dhserver.c"
    INCLUDE_DIRS "include"
                 "${TINYUSB_NETLIB_DIR}"
    REQUIRES espressif__esp_tinyusb esp_netif lwip
)

# ESP-IDF builds with warnings-as-errors; TinyUSB's dhserver.c can trigger -Waddress
# due to LWIP macros checking pointer non-nullness. Keep the upstream file intact
# but relax this warning for that translation unit only.
set_source_files_properties(
    "${TINYUSB_NETLIB_DIR}/dhserver.c"
    PROPERTIES COMPILE_FLAGS "-Wno-error=address -Wno-address"
)

idf_component_register(
    SRCS "lte_upstream_pppos.c"
         "lte_upstream_pppos_usb.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_netif esp_event freertos modem_manager esp_modem usb_dev_ethernet console
             espressif__usb_host_cdc_acm
    PRIV_REQUIRES driver hw_config usb
)

# C++17 required by lte_upstream_pppos_usb.cpp (and by esp_modem headers)
set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD          17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS        ON
)

# Expose esp_modem's private_include directory so lte_upstream_pppos_usb.cpp
# can include esp_private/c_api_wrapper.hpp (which is the intended interface for
# esp_modem DTE-extension developers, per the comment in c_api_wrapper.hpp).
idf_component_get_property(esp_modem_dir espressif__esp_modem COMPONENT_DIR)
if(esp_modem_dir)
    target_include_directories(${COMPONENT_LIB} PRIVATE
        "${esp_modem_dir}/private_include"
    )
endif()


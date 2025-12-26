idf_component_register(
    SRCS "lte_upstream_pppos.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_netif esp_event freertos modem_manager esp_modem usb_dev_ethernet
    PRIV_REQUIRES driver
)

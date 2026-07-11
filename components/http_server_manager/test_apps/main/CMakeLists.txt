idf_component_register(
    SRCS "test_http_main.c"
    REQUIRES http_server_manager filesystem esp_http_client esp_netif nvs_flash
)

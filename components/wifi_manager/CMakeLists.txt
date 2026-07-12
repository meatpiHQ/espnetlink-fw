idf_component_register(
    SRCS "wifi_manager.c"
         "wifi_manager_status.c"
         "wifi_manager_settings.c"
         "wifi_manager_select.c"
         "wifi_manager_http.c"
         "wifi_manager_cli.c"
         "wifi_manager_suspend.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # public header exposes only esp_err + FreeRTOS event-group types
    # (freertos is an implicit common requirement) -> everything is private.
    # http_server_manager is used ONLY by wifi_manager_http.c (HTTP_API.md §7
    # feature-registration); cmdline_manager only by wifi_manager_cli.c —
    # the radio code never touches either.
    PRIV_REQUIRES esp_wifi esp_netif esp_event settings_manager log_manager
                  dev_status_manager http_server_manager esp_http_server
                  cmdline_manager espressif__cjson
)

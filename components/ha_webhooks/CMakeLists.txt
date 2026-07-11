idf_component_register(
    SRCS "ha_webhooks.c"
         "ha_webhooks_settings.c"
         "ha_webhooks_http.c"
         "ha_webhooks_post.c"
         "ha_webhooks_cli.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # settings/log descriptors; the /api/webhook route; outbound POST;
    # the payload sources (dev_status, LTE link status, GPS fix, mdns
    # hostname); the console command. cJSON via the IDF 5.5 bundled json.
    PRIV_REQUIRES settings_manager log_manager http_server_manager
                  http_client_manager dev_status_manager
                  gps lte_upstream_pppos mdns_manager vpn_manager
                  cmdline_manager json
)

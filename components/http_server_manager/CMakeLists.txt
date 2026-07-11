idf_component_register(
    SRCS "http_server_manager.c"
         "http_server_manager_match.c"
         "http_server_manager_serve.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # esp_http_server is REQUIRES (not PRIV): the public header exposes
    # httpd_uri_t / httpd_handle_t.
    REQUIRES esp_http_server
    # file serving goes through the filesystem component (only the .c uses it)
    PRIV_REQUIRES filesystem log_manager
)

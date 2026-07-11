idf_component_register(
    SRCS "api_http.c"
         "api_http_util.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Glue layer: depends DOWN on the cores + the server owner (standard §6).
    # The public header exposes only esp_err_t -> everything private.
    # cJSON comes from the IDF v5.5 bundled `json` component.
    PRIV_REQUIRES http_server_manager settings_manager dev_status_manager
                  log_manager esp_http_server json
)

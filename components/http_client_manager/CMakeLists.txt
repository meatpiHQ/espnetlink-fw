idf_component_register(
    SRCS "http_client_manager.c"
         "http_client_manager_events.c"
         "http_client_manager_auth.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Service: the one way components do HTTP-client work. Per-request
    # client instances on the caller's task (thread-safe by isolation) +
    # a concurrency limiter. ESPNetLink port: event_manager glue is a
    # no-op stub; cert_manager sets are unsupported (bundle-only TLS);
    # download-to-file answers ESP_ERR_NOT_SUPPORTED (no filesystem
    # write-stream API on this product).
    PRIV_REQUIRES esp_http_client mbedtls log_manager
)
idf_component_register(
    SRCS "http_client_manager.c"
         "http_client_manager_events.c"
         "http_client_manager_auth.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Service: the one way components do HTTP-client work. Per-request
    # client instances on the caller's task (thread-safe by isolation) +
    # a concurrency limiter; TLS via cert_manager sets. ESPNetLink port:
    # event_manager glue is a no-op stub (no event_manager on this
    # product); cert_set TLS + the atomic download-to-file path are live
    # again since the cert_manager + full filesystem ports (2026-07-12).
    PRIV_REQUIRES esp_http_client mbedtls cert_manager filesystem
                  log_manager
)

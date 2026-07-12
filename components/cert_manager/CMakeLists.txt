idf_component_register(
    SRCS "cert_manager.c"
         "cert_manager_policy.c"
         "cert_manager_http.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Service: owns named TLS certificate sets on the filesystem
    # (/data/certs/<set>/); consumers (mqtt_manager cert_set, future
    # VPN/HTTPS) borrow PSRAM-cached PEM by name. /api/certs is optional
    # (main wires it); NO read-back — keys never leave the device.
    PRIV_REQUIRES filesystem http_server_manager multipart_upload
                  log_manager
)

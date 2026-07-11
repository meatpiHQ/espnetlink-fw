idf_component_register(
    SRCS "multipart_upload.c"
         "multipart_parser.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # The public header exposes httpd_req_t -> esp_http_server is public.
    REQUIRES esp_http_server
)

idf_component_register(
    SRCS "filesystem.c"
         "filesystem_settings.c"
         "filesystem_stream.c"
         "filesystem_path.c"
         "filesystem_cli.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # public header exposes only stdio/esp_err types -> everything is
    # private; cmdline_manager only by the _cli.c (`fs` command)
    # ESPNetLink port: WiCAN's `fatfs` dep dropped with the /sd capacity
    # branch (no SD slot on this product); `littlefs` is named by its
    # IDF 5.5 managed-component name joltwallet__littlefs.
    PRIV_REQUIRES joltwallet__littlefs vfs log_manager cmdline_manager
                  settings_manager
)

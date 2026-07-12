idf_component_register(
    SRCS "settings_manager.c"
         "settings_manager_boot.c"
         "settings_manager_backup.c"
         "settings_manager_schema.c"
         "settings_manager_codec.c"
         "settings_manager_fields.c"
         "settings_manager_storage.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # cJSON is a public dependency (the header exposes its types); IDF v6
    # dropped the bundled `json` component, so it comes from the
    # espressif/cjson managed component (see idf_component.yml).
    # littlefs is only used by the .c files -> keep it private (shallow graph).
    REQUIRES espressif__cjson
    PRIV_REQUIRES joltwallet__littlefs
)

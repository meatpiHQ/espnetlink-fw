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
    # cJSON is a public dependency (the header exposes its types); on IDF
    # v5.5 it comes from the bundled `json` component (IDF v6 switches to the
    # espressif/cjson managed component per the standard's v6 note).
    # littlefs is only used by the .c files -> keep it private (shallow graph).
    REQUIRES json
    PRIV_REQUIRES joltwallet__littlefs
)

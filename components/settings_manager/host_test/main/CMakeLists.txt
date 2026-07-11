# Host test compiles the pure logic (schema + codec) AND the registry
# (settings_manager.c) against an in-memory storage stub, on the linux target
# (FreeRTOS POSIX port + esp_log are available there; LittleFS is not needed).
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_schema.c"
         "test_codec.c"
         "test_registry.c"
         "test_fields.c"
         "test_storage_stub.c"
         "${comp_dir}/settings_manager.c"
         "${comp_dir}/settings_manager_boot.c"
         "${comp_dir}/settings_manager_backup.c"
         "${comp_dir}/settings_manager_schema.c"
         "${comp_dir}/settings_manager_codec.c"
         "${comp_dir}/settings_manager_fields.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    # cJSON comes from the espressif/cjson managed dep (idf_component.yml);
    # IDF v6 dropped the bundled `json` component.
    REQUIRES unity
)

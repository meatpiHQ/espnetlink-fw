# Host test for the gps settings descriptor: compiles gps_settings.c against
# the real settings_manager registry + its in-memory storage stub, on the
# linux target (no hardware, no LittleFS).
set(gps_dir "${CMAKE_CURRENT_LIST_DIR}/../..")
set(sm_dir "${CMAKE_CURRENT_LIST_DIR}/../../../settings_manager")

idf_component_register(
    SRCS "test_main.c"
         "${sm_dir}/host_test/main/test_storage_stub.c"
         "${gps_dir}/gps_settings.c"
         "${sm_dir}/settings_manager.c"
         "${sm_dir}/settings_manager_boot.c"
         "${sm_dir}/settings_manager_backup.c"
         "${sm_dir}/settings_manager_schema.c"
         "${sm_dir}/settings_manager_codec.c"
         "${sm_dir}/settings_manager_fields.c"
    INCLUDE_DIRS "${gps_dir}/include" "${gps_dir}"
                 "${sm_dir}/include" "${sm_dir}" "${sm_dir}/host_test/main"
    # cJSON from the IDF v5.5 bundled `json` component.
    REQUIRES unity json
)

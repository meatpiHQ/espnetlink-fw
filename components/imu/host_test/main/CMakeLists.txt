# Host test for the imu {cli} settings descriptor: compiles imu_settings.c
# against the real settings_manager registry + its in-memory storage stub,
# on the linux target (the ICM-42670 driver itself is hardware-bound).
set(imu_dir "${CMAKE_CURRENT_LIST_DIR}/../..")
set(sm_dir "${CMAKE_CURRENT_LIST_DIR}/../../../settings_manager")

idf_component_register(
    SRCS "test_main.c"
         "${sm_dir}/host_test/main/test_storage_stub.c"
         "${imu_dir}/imu_settings.c"
         "${sm_dir}/settings_manager.c"
         "${sm_dir}/settings_manager_boot.c"
         "${sm_dir}/settings_manager_backup.c"
         "${sm_dir}/settings_manager_schema.c"
         "${sm_dir}/settings_manager_codec.c"
         "${sm_dir}/settings_manager_fields.c"
    INCLUDE_DIRS "${imu_dir}/include"
                 "${sm_dir}/include" "${sm_dir}" "${sm_dir}/host_test/main"
    # cJSON from the IDF v5.5 bundled `json` component.
    REQUIRES unity json
)

# Host test compiles ONLY the pure layers: the activity state machine
# (time injected) and the settings migration (cJSON only).
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "${comp_dir}/imu_manager_policy.c"
         "${comp_dir}/imu_manager_migrate.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    REQUIRES unity
)

# Host test compiles ONLY the pure helpers (bit names + uptime formatting).
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_fmt.c"
         "${comp_dir}/dev_status_manager_fmt.c"
    INCLUDE_DIRS "${comp_dir}"
    REQUIRES unity
)

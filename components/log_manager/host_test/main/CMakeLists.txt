# Host test compiles ONLY the pure ring core — no esp_cache/FreeRTOS needed.
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_ring.c"
         "${comp_dir}/log_manager_ring.c"
    INCLUDE_DIRS "${comp_dir}"
    REQUIRES unity
)

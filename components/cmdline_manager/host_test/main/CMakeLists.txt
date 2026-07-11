# Host test compiles ONLY the pure line assembler — no esp_console.
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "${comp_dir}/cmdline_manager_line.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    REQUIRES unity
)

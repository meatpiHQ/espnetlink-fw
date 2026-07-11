# Host test compiles ONLY the pure match logic, so it needs neither httpd nor
# VFS and runs on the linux target. esp_http_server types come from the real
# component headers via INCLUDE_DIRS (the linux target provides esp_http_server
# as a stub through the component; if unavailable, see the shim note in README).
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_match.c"
         "${comp_dir}/http_server_manager_match.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    REQUIRES unity esp_http_server
)

# Host test compiles ONLY the pure selection/ban module — no esp_wifi/netif
# needed, so it runs on the linux target.
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_select.c"
         "${comp_dir}/wifi_manager_select.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    REQUIRES unity
)

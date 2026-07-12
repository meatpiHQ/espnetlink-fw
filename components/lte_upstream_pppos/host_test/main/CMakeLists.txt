# Host test for the lte_upstream_pppos settings descriptor: compiles
# lte_upstream_pppos_settings.c against the real settings_manager registry +
# its in-memory storage stub, on the linux target.
#
# stubs/ shadows esp_netif.h and modem_manager.h — the public header pulls
# both in, but the settings TU only needs their typedef names to exist.
set(lte_dir "${CMAKE_CURRENT_LIST_DIR}/../..")
set(sm_dir "${CMAKE_CURRENT_LIST_DIR}/../../../settings_manager")

idf_component_register(
    SRCS "test_main.c"
         "${sm_dir}/host_test/main/test_storage_stub.c"
         "${lte_dir}/lte_upstream_pppos_settings.c"
         "${sm_dir}/settings_manager.c"
         "${sm_dir}/settings_manager_boot.c"
         "${sm_dir}/settings_manager_backup.c"
         "${sm_dir}/settings_manager_schema.c"
         "${sm_dir}/settings_manager_codec.c"
         "${sm_dir}/settings_manager_fields.c"
    INCLUDE_DIRS "stubs" "${lte_dir}/include"
                 "${sm_dir}/include" "${sm_dir}" "${sm_dir}/host_test/main"
    # cJSON comes from the espressif/cjson managed dep (idf_component.yml);
    # IDF v6 dropped the bundled `json` component.
    REQUIRES unity
)

# Host test for wifi_modem: the pure helpers (wifi_modem_core.c), the
# settings descriptor (wifi_modem_settings.c) and the AP password
# provisioning (wifi_modem_provision.c) against the real settings_manager
# registry + its in-memory storage stub, on the linux target (no hardware,
# no LittleFS). The engine/DNS/HTTP/CLI files need lwIP/esp_wifi and are
# covered by test_apps/ on the device.
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")
set(sm_dir "${CMAKE_CURRENT_LIST_DIR}/../../../settings_manager")

idf_component_register(
    SRCS "test_main.c"
         "${sm_dir}/host_test/main/test_storage_stub.c"
         "${comp_dir}/wifi_modem_core.c"
         "${comp_dir}/wifi_modem_settings.c"
         "${comp_dir}/wifi_modem_provision.c"
         "${sm_dir}/settings_manager.c"
         "${sm_dir}/settings_manager_boot.c"
         "${sm_dir}/settings_manager_backup.c"
         "${sm_dir}/settings_manager_schema.c"
         "${sm_dir}/settings_manager_codec.c"
         "${sm_dir}/settings_manager_fields.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
                 "${sm_dir}/include" "${sm_dir}" "${sm_dir}/host_test/main"
    # cJSON comes from the espressif/cjson managed dep (idf_component.yml);
    # IDF v6 dropped the bundled `json` component.
    REQUIRES unity
)

# test-only reset hook in wifi_modem_settings.c (standard §3 exception)
target_compile_definitions(${COMPONENT_LIB} PRIVATE WIFI_MODEM_HOST_TEST=1)

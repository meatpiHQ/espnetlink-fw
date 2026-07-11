# Host test compiles ONLY the pure session state machine (flash ops are
# injected recorders) — no esp_ota, so it runs on the linux target.
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_session.c"
         "${comp_dir}/ota_session.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    REQUIRES unity
)

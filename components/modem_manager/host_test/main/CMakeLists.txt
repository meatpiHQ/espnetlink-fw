# Host test for modem_manager's pure AT-response parsers
# (modem_manager_parse.c) — no UART, no hardware.
set(mm_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "${mm_dir}/modem_manager_parse.c"
    INCLUDE_DIRS "${mm_dir}"
    REQUIRES unity
)

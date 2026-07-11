# Host test for api_http's pure utility layer (api_http_util.c) — the file
# is esp_http_server-free by design so it compiles unchanged on the linux
# target.
set(api_dir "${CMAKE_CURRENT_LIST_DIR}/../..")
set(rt_dir "${CMAKE_CURRENT_LIST_DIR}/../../../restart_tracker")

idf_component_register(
    SRCS "test_main.c"
         "${api_dir}/api_http_util.c"
    # restart_tracker/include: api_http_private.h pulls restart_tracker.h
    # (pure types header, host-safe)
    INCLUDE_DIRS "${api_dir}" "${rt_dir}/include"
    # cJSON from the IDF v5.5 bundled `json` component; log for esp_log.h.
    REQUIRES unity json log
)

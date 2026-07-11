# web_ui — the ESPNetLink settings/status UI (WiCAN v6 layout + branding).
# Its ES-module assets are gzipped into C byte arrays by gen_assets.py at
# build time (codegen, not EMBED_FILES, so the many files get correct MIME
# + gzip cleanly).
#
# The codegen block is guarded: IDF expands component CMakeLists in SCRIPT
# mode first (requirements pass), where add_custom_command is not
# scriptable. (WiCAN hid this behind its CONFIG_WICAN_WEBUI_V6 gate —
# config vars are undefined during that pass.)
set(srcs "web_ui.c")

if(NOT CMAKE_SCRIPT_MODE_FILE)
    idf_build_get_property(python PYTHON)
    set(gen_c "${CMAKE_CURRENT_BINARY_DIR}/web_ui_gen.c")
    file(GLOB_RECURSE web_files "${CMAKE_CURRENT_LIST_DIR}/web/*")
    add_custom_command(
        OUTPUT "${gen_c}"
        COMMAND "${python}" "${CMAKE_CURRENT_LIST_DIR}/gen_assets.py"
                "${CMAKE_CURRENT_LIST_DIR}/web" "${gen_c}"
        DEPENDS "${CMAKE_CURRENT_LIST_DIR}/gen_assets.py" ${web_files}
        COMMENT "Generating web UI asset blobs"
        VERBATIM)
    list(APPEND srcs "${gen_c}")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES http_server_manager
)

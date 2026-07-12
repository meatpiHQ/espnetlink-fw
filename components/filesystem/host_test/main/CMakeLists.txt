# Host test compiles ONLY the pure path module — no VFS/littlefs needed,
# so it runs on the linux target.
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_path.c"
         "${comp_dir}/filesystem_path.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    REQUIRES unity
)

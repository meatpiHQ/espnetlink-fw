# Build & run on the host:
#   idf.py --preview set-target linux
#   idf.py build
#   ./build/lte_upstream_pppos_host_test.elf   # exits non-zero if a test fails
cmake_minimum_required(VERSION 3.16)
# restrict the build to main + its deps (drivers don't resolve on linux)
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lte_upstream_pppos_host_test)

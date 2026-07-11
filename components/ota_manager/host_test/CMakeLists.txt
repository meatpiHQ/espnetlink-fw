# Build & run on the host:
#   idf.py --preview set-target linux
#   idf.py build
#   ./build/ota_manager_host_test.elf       # exits non-zero if a test fails
cmake_minimum_required(VERSION 3.16)
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ota_manager_host_test)

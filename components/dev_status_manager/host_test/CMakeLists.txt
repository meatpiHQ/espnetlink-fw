# Build & run on the host:
#   idf.py --preview set-target linux
#   idf.py build
#   ./build/dev_status_manager_host_test.elf   # exits non-zero if a test fails
cmake_minimum_required(VERSION 3.16)
# restrict the build to main + its deps (drivers don't resolve on linux)
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dev_status_manager_host_test)

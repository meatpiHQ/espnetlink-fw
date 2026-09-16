# Build & run on the host:
#   idf.py --preview set-target linux && idf.py build
#   ./build/led_manager_host_test.elf
cmake_minimum_required(VERSION 3.16)
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(led_manager_host_test)

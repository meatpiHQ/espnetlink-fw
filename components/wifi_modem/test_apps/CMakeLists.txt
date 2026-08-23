cmake_minimum_required(VERSION 3.16)
# ESPNetLink components root: the component under test plus its deps
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")
# Standard §7: inherit the MAIN firmware's config, then the small test-only
# overlay in this directory (partition path + console baud only).
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../../../sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wifi_modem_test_app)

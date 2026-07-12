cmake_minimum_required(VERSION 3.16)
# the component under test lives one level up
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/..")
# Standard §7 (rev 2.1): inherit the MAIN firmware's config, then the
# small test-only overlay in this directory (pins esp32s3 + the product
# partition table; without it a fresh configure defaults to esp32).
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../../../sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(settings_manager_test_app)

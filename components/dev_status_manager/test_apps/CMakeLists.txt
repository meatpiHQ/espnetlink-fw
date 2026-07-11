cmake_minimum_required(VERSION 3.16)
# WiCAN components root: the component under test plus its deps (log_manager)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")
# Standard §7 (rev 2.1): inherit the MAIN firmware's config, then the
# small test-only overlay in this directory. (Retrofitted 2026-07-06:
# without this, a reconfigure regenerates sdkconfig WITHOUT newer root
# options — e.g. HTTPD_WS_SUPPORT — and components added since then
# fail to compile, because every component in components/ builds in
# every test app.)
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../../../sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dev_status_manager_test_app)

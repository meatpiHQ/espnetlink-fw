# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(GIT_SHA1 "${GIT_SHA}")
string(REPLACE "." "" GIT_SHA "${GIT_SHA}")

add_definitions(-DGIT_SHA="${GIT_SHA1}")

# Form the project binary name
set(PRJ_BIN "espnetlink-fw_${GIT_SHA}")

# CherryUSB (managed component) does not enable any device classes by default on ESP-IDF.
# Enable Device + RNDIS so `usbd_rndis.c` is compiled and available.
set(CONFIG_CHERRYUSB_DEVICE 1)
set(CONFIG_CHERRYUSB_DEVICE_CDC_RNDIS 1)

# Enable CherryUSB's optional lwIP helpers: `usbd_rndis_eth_rx/tx()`.
add_compile_definitions(CONFIG_USBDEV_RNDIS_USING_LWIP)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project("${PRJ_BIN}")

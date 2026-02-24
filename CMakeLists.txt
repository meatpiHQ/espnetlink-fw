# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Hardware version definitions
set(ESPNETLINK_V100 1)

set(ESPNETLINK_V100_STR "ESPNETLINK")

set(ESPNETLINK_V100_PREF "espnetlink")

add_definitions(-DESPNETLINK_V100=${ESPNETLINK_V100})

add_definitions(-DESPNETLINK_V100_STR="${ESPNETLINK_V100_STR}")

execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(GIT_SHA1 "${GIT_SHA}")
string(REPLACE "." "" GIT_SHA "${GIT_SHA}")

add_definitions(-DGIT_SHA="${GIT_SHA1}")

set(HARDWARE_VER ${ESPNETLINK_V100})

add_definitions(-DHARDWARE_VER=${HARDWARE_VER})

set(HW_PREF "${ESPNETLINK_V100_PREF}")
set(HW_VER "${ESPNETLINK_V100_STR}")

add_definitions(-DHARDWARE_VERSION="${HW_VER}")

set(PROJECT_PREF "espnetlink-fw")

# Form the project binary name
set(PRJ_BIN "${PROJECT_PREF}_${HW_PREF}_${GIT_SHA}")

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

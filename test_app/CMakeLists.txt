cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
	"../components/config_manager"
	"../components/config_manager/test"
	"../components/filesystem"
	"../managed_components/joltwallet__littlefs"
)
set(TEST_COMPONENTS config_manager filesystem)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(unit-test-app)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "DO NOT BUILD in-tree.")
endif()

cmake_minimum_required(VERSION 3.1)
project(pi_bno055 VERSION 1.0)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file(pi_bno055Config.h.in pi_bno055Config.h)

set(CMAKE_C_FLAGS "-O3 -Wall -g")
# set vars for installation dest
set(example_dest bin/${CMAKE_PROJECT_NAME})
set(include_dest include/${CMAKE_PROJECT_NAME})
set(main_lib_dest lib/${CMAKE_PROJECT_NAME})
set(main_lib_cmake_dest lib/${CMAKE_PROJECT_NAME}/cmake/getbno055)
add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_subdirectory(src)
#add_subdirectory(${CMAKE_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME})
#set(example_source ${CMAKE_SOURCE_DIR}/example/getbno055.c)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/example example_source)
add_executable(getbno055_tool ${example_source})
#Comment: The signature e.g. LINK_PUBLIC or LINK_PRIVATE is for compatibility only.
# Prefer the PUBLIC or PRIVATE keywords instead.
target_link_libraries(getbno055_tool PRIVATE getbno055)
# add the binary tree to the search path for include files
# so that we will find pi_bno055Config.h
target_include_directories(getbno055_tool PUBLIC
        "${PROJECT_BINARY_DIR}"
        )

#########################
## Install and Package ##
#########################
message(STATUS "the dafault root of installation is ${CMAKE_INSTALL_PREFIX}. To be altered if needed.")
install(TARGETS getbno055_tool
        DESTINATION ${example_dest})
install(FILES "${PROJECT_BINARY_DIR}/pi_bno055Config.h"
        DESTINATION ${include_dest}
        )

install(EXPORT getbno055
        FILE getbno055.cmake
        DESTINATION ${main_lib_cmake_dest})
# generate the package configuration file that is includes the exports
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/getbno055Config.cmake"
        INSTALL_DESTINATION "lib/cmake/example"
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
        )
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/getbno055ConfigVersion.cmake"
        VERSION "${pi_bno055_VERSION_MAJOR}.${pi_bno055_VERSION_MINOR}"
        COMPATIBILITY AnyNewerVersion
)
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/getbno055Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/getbno055ConfigVersion.cmake
        DESTINATION ${main_lib_cmake_dest}
        )
export(EXPORT getbno055
        FILE "${CMAKE_CURRENT_BINARY_DIR}/getbno055.cmake"
        )
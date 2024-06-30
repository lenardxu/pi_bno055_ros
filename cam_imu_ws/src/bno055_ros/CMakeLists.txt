cmake_minimum_required(VERSION 3.1)
project(bno055_ros)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  roslib
  tf2
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)
if(${CMAKE_VERSION} VERSION_LESS "3.19.0")
    message(STATUS "Current CMake Version: ${CMAKE_VERSION}")
#    set(getbno055_DIR "/home/rukangxu/MA_IAS/HandTracker/utils/pi_bno055/install/lib/pi_bno055/cmake/getbno055")
    get_filename_component(getbno055_REAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../pi_bno055/install/lib/pi_bno055/cmake/getbno055" REALPATH)
    message(STATUS "the dir including getbno055 config cmake file is ${getbno055_REAL_DIR}")
    set(getbno055_DIR ${getbno055_REAL_DIR})
else()
    message(STATUS "Current CMake Version: ${CMAKE_VERSION}")
    file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../pi_bno055/install/lib/pi_bno055/cmake/getbno055" getbno055_REAL_DIR)
    message(STATUS "the dir including getbno055 config cmake file is ${getbno055_REAL_DIR}")
    set(getbno055_DIR ${getbno055_REAL_DIR})
endif()
find_package(getbno055 CONFIG REQUIRED)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bno055_ros
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(matplotplusplus)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(bno055_pub_node src/bno055_pub_node.cpp src/bno055_pub.cpp src/bno055_utils.cpp)
target_link_libraries(bno055_pub_node ${catkin_LIBRARIES} getbno055)

add_executable(bno055_sub_node src/bno055_sub_node.cpp src/bno055_sub.cpp)
target_include_directories(bno055_sub_node PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(bno055_sub_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(imu_tf_broadcaster_node src/imu_tf_broadcaster.cpp)
target_link_libraries(imu_tf_broadcaster_node ${catkin_LIBRARIES})

add_executable(plot_imu_data_node src/plot_imu_data_node.cpp src/plot_imu_data.cpp)
target_include_directories(plot_imu_data_node PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(plot_imu_data_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} matplot)

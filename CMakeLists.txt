cmake_minimum_required(VERSION 2.8.3)
project(gnss_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  sensor_msgs
  tf
  gbx_ros_bridge_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS gbx_ros_bridge_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(gnss_visualization
               src/gnss_visualization.cpp
               src/helper.cpp)
target_link_libraries(gnss_visualization ${catkin_LIBRARIES})
add_dependencies(gnss_visualization ${catkin_EXPORTED_TARGETS})

add_dependencies(gnss_visualization gbx_ros_bridge_msgs_gencpp)


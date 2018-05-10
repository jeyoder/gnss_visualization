cmake_minimum_required(VERSION 2.8.3)
project(gnss_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Qt stuff (needed for an rviz plugin)

find_package(catkin REQUIRED)


## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
#if(rviz_QT_VERSION VERSION_LESS "5")
#  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#  ## pull in all required include dirs, define QT_LIBRARIES, etc.
#  include(${QT_USE_FILE})
#else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
#  include('AFDSJDFKJSKFDKJSDF')
#endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

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
  rviz
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

##### gnss_visualization ROS node #####
add_executable(gnss_visualization
               src/gnss_visualization.cpp
               src/arena_viz.cpp
               src/quadcopter_viz.cpp
               src/helper.cpp)
target_link_libraries(gnss_visualization ${catkin_LIBRARIES})
add_dependencies(gnss_visualization ${catkin_EXPORTED_TARGETS})

add_dependencies(gnss_visualization gbx_ros_bridge_msgs_gencpp)


##### gnss_visualization_camera_controller rviz plugin #####
add_library(gnss_visualization_camera_controller
    src/camera_controller.cpp include/gnss_visualization/camera_controller.h)
target_link_libraries(gnss_visualization_camera_controller ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gnss_visualization_camera_controller ${catkin_EXPORTED_TARGETS})

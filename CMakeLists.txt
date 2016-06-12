cmake_minimum_required(VERSION 2.8.3)
project(netusb_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  sensor_msgs
)

#generate_dynamic_reconfigure_options(
#)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure image_transport nodelet roscpp sensor_msgs
)

message(STATUS "finding libNETUSBCAM")
find_library(NETUSBCAM_LIB NETUSBCAM)
if(NOT NETUSBCAM_LIB)
  message(STATUS "libNETUSBCAM not found in system library path")
  include(cmake/DownloadNETUSBCAM.cmake)
  download_netusbcam(NETUSBCAM_LIB NETUSBCAM_INCLUDE_DIR)
endif()

if(NETUSBCAM_LIB)
  message(STATUS "NETUSBCAM library: ${NETUSBCAM_LIB}")
  message(STATUS "NETUSBCAM include: ${NETUSBCAM_INCLUDE_DIR}")
  include_directories(${NETUSBCAM_INCLUDE_DIR})
else()
  message(FATAL_ERROR "NETUSBCAM not found")
endif()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(NETUSBCamera src/NETUSBCamera.cpp)
target_link_libraries(NETUSBCamera ${NETUSBCAM_LIB} ${catkin_LIBRARIES})

add_executable(netusb_camera_node src/netusb_camera_node.cpp)
target_link_libraries(netusb_camera_node NETUSBCamera ${catkin_LIBRARIES})

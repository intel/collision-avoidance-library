cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
  DepthCamera.cc
  GazeboRealSenseCamera.cc
  RealSenseCamera.cc
)

set (headers
  DepthCamera.hh
  GazeboRealSenseCamera.hh
  RealSenseCamera.hh
)

add_library(sensors ${sources})
target_link_libraries(sensors communication ${REALSENSE_LIBRARIES} ${GAZEBO_LIBRARIES} ${LDFLAGS})


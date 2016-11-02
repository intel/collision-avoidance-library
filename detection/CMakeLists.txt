cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
  DepthImageObstacleDetector.cc
  DepthImageStraightLineDetector.cc
  DepthImagePolarHistDetector.cc
  DepthImageSimpleDetector.cc
)

add_library(detection ${sources})
target_link_libraries(detection common utils ${LDFLAGS})


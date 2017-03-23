cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES
    DepthImageObstacleDetector.cc
    DepthImageStraightLineDetector.cc
    DepthImagePolarHistDetector.cc)

add_library(detection OBJECT ${SOURCES})

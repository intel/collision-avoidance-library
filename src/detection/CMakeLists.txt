cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES
    DepthImageObstacleDetector.cc
    DepthImagePolarHistDetector.cc
    DepthImageStraightLineDetector.cc)

set(HEADERS
    Detectors.hh
    DepthImageObstacleDetector.hh
    DepthImagePolarHistDetector.hh
    DepthImageStraightLineDetector.hh)

export_headers("${HEADERS}" "detection")
set(COAV_INCLUDE_LIST "${COAV_INCLUDE_LIST}${INCLUDE_LIST}" PARENT_SCOPE)

add_library(detection OBJECT ${SOURCES})

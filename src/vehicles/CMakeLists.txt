cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES MavQuadCopter.cc)
set(HEADERS
    Vehicles.hh
    MavQuadCopter.hh)

export_headers("${HEADERS}" "vehicles")
set(COAV_INCLUDE_LIST "${COAV_INCLUDE_LIST}${INCLUDE_LIST}" PARENT_SCOPE)

add_library(vehicles OBJECT ${SOURCES})

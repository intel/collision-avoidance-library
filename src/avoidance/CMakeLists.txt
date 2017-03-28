cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES
    QuadCopterShiftAvoidance.cc
    QuadCopterStopAvoidance.cc
    QuadCopterVFFAvoidance.cc)

set(HEADERS
    Avoidance.hh
    QuadCopterShiftAvoidance.hh
    QuadCopterStopAvoidance.hh
    QuadCopterVFFAvoidance.hh)

export_headers("${HEADERS}" "avoidance")
set(COAV_INCLUDE_LIST "${COAV_INCLUDE_LIST}${INCLUDE_LIST}" PARENT_SCOPE)

add_library(avoidance OBJECT ${SOURCES})

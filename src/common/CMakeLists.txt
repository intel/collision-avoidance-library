cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES
    common.cc
    math.cc)

set(HEADERS common.hh)

export_headers("${HEADERS}" "common")
set(COAV_INCLUDE_LIST "${COAV_INCLUDE_LIST}${INCLUDE_LIST}" PARENT_SCOPE)

add_library(common OBJECT ${SOURCES})

cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES
    common.cc
    math.cc)

add_library(common OBJECT ${SOURCES})

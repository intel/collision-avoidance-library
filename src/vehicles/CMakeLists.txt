cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES MavQuadCopter.cc)

add_library(vehicles OBJECT ${SOURCES})

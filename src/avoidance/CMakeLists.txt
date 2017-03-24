cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES
    QuadCopterShiftAvoidance.cc
    QuadCopterStopAvoidance.cc
    QuadCopterVFFAvoidance.cc)

add_library(avoidance OBJECT ${SOURCES})

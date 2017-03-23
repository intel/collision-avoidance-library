cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set (sources
  math.cc
  DepthCamera.cc)

add_library(common OBJECT ${sources})

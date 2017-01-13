cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set (sources
  PPTree.cc
  DebugUtils.cc
)

add_library(utils ${sources})

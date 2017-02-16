cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(SOURCES MavQuadCopter.cc)
set(LIBRARIES mavlink_vehicles)

add_library(vehicles ${SOURCES})
target_link_libraries(vehicles PRIVATE ${LIBRARIES})

cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES
  GazeboRealSenseCamera.cc
)

set(LIBRARIES
    communication
    ${GAZEBO_LIBRARIES}
)

if(REALSENSE_FOUND)
    set(SOURCES ${SOURCES} RealSenseCamera.cc)
    set(LIBRARIES ${LIBRARIES} ${REALSENSE_LIBRARIES})
endif(REALSENSE_FOUND)

add_library(sensors ${SOURCES})
target_link_libraries(sensors ${LIBRARIES} ${LDFLAGS})


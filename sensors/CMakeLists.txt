cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(SOURCES
  GazeboRealSenseCamera.cc
)

set(LIBRARIES
    communication
    ${GAZEBO_LIBRARIES}
)

set(INCLUDES
)

if(REALSENSE_FOUND)
    set(SOURCES ${SOURCES} RealSenseCamera.cc)
    set(LIBRARIES ${LIBRARIES} ${REALSENSE_LIBRARIES})
    set(INCLUDES ${INCLUDES} ${REALSENSE_INCLUDE_DIRS})
endif()

add_library(sensors ${SOURCES})
target_link_libraries(sensors ${LIBRARIES} ${LDFLAGS})
target_include_directories(sensors PRIVATE ${INCLUDES})


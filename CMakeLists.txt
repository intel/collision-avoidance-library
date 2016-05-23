cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(LIBCOAV)

# Add module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Compiler flags
set(CUSTOM_COMPILE_FLAGS "-Wall -g")
set(LDFLAGS "-z noexecstack -z relro -z now")
set(CFLAGS "-fstack-protector-all -fPIE -fPIC -O2 -D_FORTIFY_SOURCE=2")
set(CFLAGS "${CFLAGS} -Wformat -Wformat-security")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS} ${CUSTOM_COMPILE_FLAGS}")
set(CMAKE_CXX_STANDARD 11)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)
find_package(GLM REQUIRED)
find_package(OpenGL)
find_package(GLUT)

include_directories(${PROJECT_SOURCE_DIR} ${GAZEBO_INCLUDE_DIRS} interfaces)
link_directories(${GAZEBO_LIBRARY_DIRS})

# Add library subdirectories
add_subdirectory(avoidance)
add_subdirectory(common)
add_subdirectory(communication)
add_subdirectory(detection)
add_subdirectory(samples)
add_subdirectory(sensors)
add_subdirectory(tests)
add_subdirectory(utils)
add_subdirectory(vehicles)

# Add gazebo plugins and models subdirectories
add_subdirectory(modules/gazebo_realsense)
add_subdirectory(modules/mavlink_vehicles)
add_subdirectory(simulation/gazebo-models)

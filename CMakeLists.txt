cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(LIBCOAV)

#Set add REQUIRED list to DEP_VAR if FEATURE is ON
function(option_deps DEP_VAR FEATURE REQUIRED)
    if (${FEATURE})
        set(${DEP_VAR} ${${DEP_VAR}} ${REQUIRED} ${ARGN} PARENT_SCOPE)
    endif()
endfunction()

#add option dependency
macro(dependent_option option doc default dependency)
    if (${dependency})
        set(enable_${option} true)
    endif()

    if (DEFINED enable_${option})
        option(${option} ${doc} ${default})
    else()
        set(${option} OFF)
        message(WARNING "Option ${option} ignored. Missing required option ${dependency}")
    endif()
endmacro()

#Build Options
option(WITH_REALSENSE "Add realsense support" ON)
option(WITH_GAZEBO "Enable Gazebo simulation features" OFF)
dependent_option(WITH_VDEBUG "Compile visual debugger" OFF WITH_GAZEBO)

# Check if submodules have been initialized
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink_vehicles/CMakeLists.txt)
    message(SEND_ERROR "mavlink_vehicles submodule not found")
    return()
endif()

#List required packages for each build option
set(DEPS "GLM")
option_deps(DEPS WITH_GAZEBO "gazebo")
option_deps(DEPS WITH_REALSENSE "realsense")
option_deps(DEPS WITH_VDEBUG "gazebo" "OpenGL" "GLUT")
list(REMOVE_DUPLICATES DEPS)

#List optional packages
set(OPTIONAL_DEPS "Doxygen")
list(REMOVE_DUPLICATES OPTIONAL_DEPS)

# Add module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find required packages
foreach(dep ${DEPS})
    find_package(${dep} REQUIRED)
endforeach()

# Find optional packages
foreach(dep ${OPTIONAL_DEPS})
    find_package(${dep})
endforeach()

# Generate documentation
if(DOXYGEN_FOUND)
    set(GENERATED_DOC_DIR "${CMAKE_CURRENT_BINARY_DIR}/doc")
    file(MAKE_DIRECTORY ${GENERATED_DOC_DIR})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)
    add_custom_target(doc
                      ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                      WORKING_DIRECTORY ${GENERATED_DOC_DIR}
                      COMMENT "Generating API documentation" VERBATIM)
else()
    message(WARNING "Doxygen has not been found. Install it and run cmake again \
    to generate the documentation with 'make doc'. Please ignore this message \
    if you're not planning to generate the Doxygen documentation right now.")
endif(DOXYGEN_FOUND)

# Compiler flags
set(CMAKE_CXX_STANDARD 11)
set(CUSTOM_COMPILE_FLAGS "${CUSTOM_COMPILE_FLAGS} -Wall")
set(CFLAGS "${CFLAGS} -fstack-protector-all -fPIE -fPIC -O2 -D_FORTIFY_SOURCE=2")
set(CFLAGS "${CFLAGS} -Wformat -Wformat-security")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_COMPILE_FLAGS}")
set(CMAKE_STATIC_LINKER_FLAGS "${LDFLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${LDFLAGS} -z noexecstack -z relro -z now")
set(CMAKE_EXE_LINKER_FLAGS "${LDFLAGS} -z noexecstack -z relro -z now")

# Include and Link directories
include_directories(${PROJECT_SOURCE_DIR})

if (${WITH_GAZEBO})
    # Custom instalation paths
    if (COAVLIB_PLUGIN_INSTALL_PATH)
        GET_FILENAME_COMPONENT(COAVLIB_PLUGIN_PATH ${COAVLIB_PLUGIN_INSTALL_PATH} ABSOLUTE)
    else()
        set(COAVLIB_PLUGIN_PATH ${GAZEBO_PLUGIN_PATH})
    endif()

    # Gazebo 7 does not provide a ${GAZEBO_MODEL_PATH} variable.
    # That's why we are accessing it relatively to ${GAZEBO_MEDIA_PATH}.
    # A proper ${GAZEBO_MODEL_PATH} variable might be available in Gazebo 8+.
    if (COAVLIB_MODEL_INSTALL_PATH)
        GET_FILENAME_COMPONENT(COAVLIB_MODEL_PATH ${COAVLIB_MODEL_INSTALL_PATH} ABSOLUTE)
    elseif(GAZEBO_MODEL_PATH)
        set(COAVLIB_MODEL_PATH ${GAZEBO_MODEL_PATH})
    else()
        GET_FILENAME_COMPONENT(COAVLIB_MODEL_PATH ${GAZEBO_MEDIA_PATH}/../models ABSOLUTE)
    endif()

    # Add submodules and gazebo plugins subdirectories
    set(GZRS_PLUGIN_INSTALL_PATH ${COAVLIB_PLUGIN_PATH})
    set(GZRS_MODEL_INSTALL_PATH ${COAVLIB_MODEL_PATH})

    # Include and Link directories
    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
endif()

if (${WITH_REALSENSE})
    include_directories(${REALSENSE_INCLUDE_DIRS})
endif()

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
add_subdirectory(modules/mavlink_vehicles)

if (${WITH_GAZEBO})
    add_subdirectory(modules/gazebo_realsense)
    add_subdirectory(simulation/gazebo-models)
endif()

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION $ENV{HOME}/.gazebo/models
        USE_SOURCE_PERMISSIONS
        PATTERN "CMakeLists.txt" EXCLUDE)


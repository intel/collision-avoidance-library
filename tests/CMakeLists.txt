cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(unity_tests Tests.cc)
add_library(pptree_tests PPTree_Test.cc)


target_link_libraries(pptree_tests
    utils
    ${GAZEBO_libraries}
    ${LDFLAGS}
)

target_link_libraries(unity_tests
    pptree_tests
    ${GAZEBO_libraries}
    ${LDFLAGS}
)

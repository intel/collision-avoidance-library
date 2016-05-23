cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(tests Tests.cc)
add_library(pptree_tests PPTree_Test.cc)


target_link_libraries(pptree_tests
    utils
    ${GAZEBO_libraries}
    ${LDFLAGS}
)

target_link_libraries(tests
    pptree_tests
    ${GAZEBO_libraries}
    ${LDFLAGS}
)

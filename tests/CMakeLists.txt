cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

add_executable(unit_tests Tests.cc)
add_library(pptree_tests PPTree_Test.cc)


target_link_libraries(pptree_tests
    utils
)

target_link_libraries(unit_tests
    pptree_tests
)

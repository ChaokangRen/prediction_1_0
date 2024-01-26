cmake_minimum_required(VERSION 3.0.2)
project(predictor)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
    eigen3
)
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/prediction
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/opt
    ${PC_DEPEND_INCLUDE_DIRS}
)

# # Build
add_library(${PROJECT_NAME} STATIC
    free_move_predictor.cpp
    lane_sequence_predictor.cpp
    junction_predictor.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
    common
)
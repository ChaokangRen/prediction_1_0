cmake_minimum_required(VERSION 3.0.2)
project(container)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
)
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/prediction
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/common
    ${PC_DEPEND_INCLUDE_DIRS}
)

# # Build
add_library(${PROJECT_NAME} STATIC
    lane_container.cpp    
    obstacle_container.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
    common
)
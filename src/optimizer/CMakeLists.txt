cmake_minimum_required(VERSION 3.0.2)

project(optimizer)

find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
  sglog
  sgtime
  eigen3
)

include_directories(
  include
  ${PC_DEPEND_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} STATIC
  optObjTraj.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PC_DEPEND_LIBRARIES}
  ${Boost_LIBRARIES}
  -lgtsam
  -ltbb
)

# install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

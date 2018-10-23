cmake_minimum_required(VERSION 2.8.3)
project(reactive_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    tf
    stdr_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

add_executable(robot
  src/main.cpp
  src/reactive_robot.cpp)
add_dependencies(robot stdt_msgs_gencpp)

target_link_libraries(robot ${catkin_LIBRARIES})

install(TARGETS robot
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

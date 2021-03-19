cmake_minimum_required(VERSION 2.8.3)
project(cluster_polygon)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  interactive_markers
  roslib
  sg_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_polygon
  src/cluster_polygon.cpp

)

target_link_libraries(cluster_polygon
  ${catkin_LIBRARIES}
)



cmake_minimum_required(VERSION 2.8.3)
project(guiding_utils)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(PCL REQUIRED)
FIND_PACKAGE(RTABMap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  message_filters
  image_transport
  cv_bridge
  people_msgs
  spencer_tracking_msgs
  cob_perception_msgs
  pcl_conversions
  pcl_ros
  cmr_msgs
  tf_conversions
  sg_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(message_republisher
  src/message_republisher.cpp
  include/rotate_image_pet/message_republisher.h
)

add_executable(rotate_image
  src/rotate_image.cpp
)



target_link_libraries(rotate_image
  ${catkin_LIBRARIES}
)

target_link_libraries(message_republisher
  ${catkin_LIBRARIES}
)




add_dependencies(message_republisher ${PROJECT_NAME}_gencfg)
add_dependencies(message_republisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(rotate_image ${PROJECT_NAME}_gencfg)
add_dependencies(rotate_image ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(detection_com
  src/detection_com.cpp
  include/detection_com.h
)

target_link_libraries(detection_com
  ${catkin_LIBRARIES}
)



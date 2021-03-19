cmake_minimum_required(VERSION 2.8.3)
project(object3d_detector)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs geometry_msgs pcl_conversions pcl_ros spencer_tracking_msgs sg_msgs)

find_package(PCL REQUIRED)
find_package(Curses REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${CURSES_INCLUDE_DIR})
catkin_package()

catkin_package(
  INCLUDE_DIRS include)

### Offline version
add_executable(object3d_detector src/object3d_detector.cpp src/svm.cpp)
target_link_libraries(object3d_detector ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector ${catkin_EXPORTED_TARGETS})
endif()


add_executable(cluster_node src/cluster_node.cpp src/svm.cpp)
target_link_libraries(cluster_node ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(cluster_node ${catkin_EXPORTED_TARGETS})
endif()

add_executable(object3d_detector_fast src/object3d_detector_fast.cpp src/svm.cpp)
target_link_libraries(object3d_detector_fast ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector_fast ${catkin_EXPORTED_TARGETS})
endif()


#add_executable(show_pcd src/show_pcd.cpp )
#target_link_libraries(show_pcd ${catkin_LIBRARIES})
#target_link_libraries(show_pcd ${CURSES_LIBRARIES})
#if(catkin_EXPORTED_TARGETS)
#  add_dependencies(show_pcd ${catkin_EXPORTED_TARGETS})
#endif()

add_executable(training_ob_multi src/training_ob_multi.cpp src/svm.cpp)
target_link_libraries(training_ob_multi ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(training_ob_multi ${catkin_EXPORTED_TARGETS})
endif()

add_executable(training_ob_multi_fast src/training_ob_multi_fast.cpp src/svm.cpp)
target_link_libraries(training_ob_multi_fast ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(training_ob_multi_fast ${catkin_EXPORTED_TARGETS})
endif()

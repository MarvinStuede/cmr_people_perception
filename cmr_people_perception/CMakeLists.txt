cmake_minimum_required(VERSION 2.8.3)
project(cmr_people_perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp spencer_tracking_msgs)




include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

option(DOWNLOAD_BAG_DATA "Download bag files with data for tests"  OFF)

if(CATKIN_ENABLE_TESTING)
 if(DOWNLOAD_BAG_DATA)
  execute_process(COMMAND rosrun cmr_people_perception get_bag.sh)
 endif()
 find_package(rostest REQUIRED)
 add_rostest_gtest(test_people_perception
  test/people_perception.test
  test/test_people_perception.cpp
 )
 target_link_libraries(
  test_people_perception
  ${catkin_LIBRARIES}
 )
 add_dependencies(
  test_people_perception
  ${catkin_EXPORTED_TARGETS}
 )
endif()




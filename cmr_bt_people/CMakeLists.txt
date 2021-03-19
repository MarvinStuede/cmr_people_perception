cmake_minimum_required(VERSION 2.8.3)
project(cmr_bt_people)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    spencer_tracking_msgs
    cmr_bt_generic
    cmr_os
    behaviortree_cpp_v3
    tf
)



include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cmr_bt_people_conditions
    CATKIN_DEPENDS behaviortree_cpp_v3 cmr_os
    )

function(set_plugin_properties plugin)
    target_link_libraries(${plugin} PRIVATE
      ${BEHAVIOR_TREE_LIBRARY}
      ${catkin_LIBRARIES}
    )
    target_include_directories(${plugin} PUBLIC ${catkin_INCLUDE_DIRS})
    target_compile_definitions(${plugin} PRIVATE  BT_PLUGIN_EXPORT )
    set_target_properties(${plugin} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
endfunction()

add_library(cmr_bt_people_conditions SHARED
    include/cmr_bt_people/conditions.h
    src/conditions.cpp
)

set_plugin_properties(cmr_bt_people_conditions)

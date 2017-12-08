cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_rrt)
add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED 
  COMPONENTS
    nav_core
    roscpp
    rospy
    std_msgs
    angles
    costmap_2d
    geometry_msgs
    nav_msgs
    pluginlib
    tf
    move_base
    base_local_planner
)

catkin_package(
	CATKIN_DEPENDS
    nav_core
    roscpp
    rospy
    std_msgs
    angles
    costmap_2d
    geometry_msgs
    nav_msgs
    pluginlib
    tf
    move_base
    base_local_planner
)

# Build
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
)


add_library(turtlebot_rrt_lib src/turtlebot_rrt.cc)

install(FILES rrt_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Testing
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(rrt_test test/turtlebot_rrt_test.cc)
	target_link_libraries(rrt_test ${catkin_LIBRARIES})
endif()
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_rrt)
add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	nav_core
	nav_msgs
	pluginlib
	costmap_2d
	global_planner
	roscpp
	rospy
	std_msgs
	tf
	visualization_msgs
)

find_package(cmake_modules REQUIRED)

FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

catkin_package(
	CATKIN_DEPENDS
		geometry_msgs
		nav_core
		nav_msgs
		pluginlib
		costmap_2d
		global_planner
		roscpp
		rospy
		std_msgs
		tf
		visualization_msgs
	
	LIBRARIES
		turtlebot_rrt
)

# Build
include_directories(
	${Eigen_INCLUDE_DIRS}
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include/
	${CMAKE_CURRENT_SOURCE_DIR}/src/
	${CMAKE_CURRENT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
)

# Set sources variables
set(
	SOURCES_RRT
		src/${PROJECT_NAME}.cc
		include/${PROJECT_NAME}/${PROJECT_NAME}.h
)


# Plugin
add_library(
	${PROJECT_NAME} ${SOURCES_RRT}
)

target_link_libraries(
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${BOOST_LIBRARIES}
)

add_dependencies(
	${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
)

add_dependencies(
	${PROJECT_NAME}
	nav_msgs_gencpp
)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ${PROJECT_NAME} rrt_planner_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Testing
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(rrt_test test/turtlebot_rrt_test.cc)
	target_link_libraries(rrt_test ${catkin_LIBRARIES})
endif()

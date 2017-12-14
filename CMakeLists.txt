cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_rrt)
add_definitions(-std=c++11 -Wall -Werror)

option(COVERAGE "Generate Coverage Data" OFF)

if (COVERAGE)
    include(CodeCoverage)
    message(STATUS "Enabled generation of code coverage information.")
    set(COVERAGE_SRCS src/turtlebot_rrt.cc src/vertex.cc include/turtlebot_rrt/turtlebot_rrt.h include/turtlebot_rrt/vertex.h)
    set(CMAKE_CXX_FLAGS "-fprofile-arc -ftest-coverage ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    tf
    nav_core
    costmap_2d
    base_local_planner
    pluginlib
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pluginlib
    costmap_2d
    base_local_planner
    nav_core
    visualization_msgs
)

add_library(${PROJECT_NAME} 
  src/turtlebot_rrt.cc
  src/vertex.cc
)

add_dependencies(turtlebot_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES rrt_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)  

# Testing
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(
  rrt_test 
    test/turtlebotrrt_test.cc
    test/vertex_test.cc
    test/rrtpath_test.cc
  )
	target_link_libraries(rrt_test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

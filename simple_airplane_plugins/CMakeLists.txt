cmake_minimum_required(VERSION 2.8.3)
project(simple_airplane_plugins)

add_definitions(-std=c++11)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf
  pluginlib
  urdf
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES gazebo_world_plugin_loader
  CATKIN_DEPENDS roscpp tf 
    urdf
  DEPENDS gazebo 
)

add_library(gazebo_ros_planar_move_with_banking_plugin1 src/gazebo_ros_planar_move_with_banking.cpp)
target_link_libraries(gazebo_ros_planar_move_with_banking_plugin1 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_ros_planar_move_with_banking_plugin1 ${catkin_EXPORTED_TARGETS})


add_library(gazebo_ros_planar_move_with_banking_and_pitching_plugin1 src/gazebo_ros_planar_move_with_banking_and_pitching.cpp)
target_link_libraries(gazebo_ros_planar_move_with_banking_and_pitching_plugin1 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_ros_planar_move_with_banking_and_pitching_plugin1 ${catkin_EXPORTED_TARGETS})

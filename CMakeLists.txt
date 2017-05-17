cmake_minimum_required(VERSION 2.8.3)
project(my_robot_urdf_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(parser src/parser.cpp)
target_link_libraries(parser ${catkin_LIBRARIES})

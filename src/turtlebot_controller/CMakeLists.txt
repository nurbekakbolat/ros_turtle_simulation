cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  turtlesim
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_controller
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_square src/turtle_listener.cpp)

target_link_libraries(turtle_square
  ${catkin_LIBRARIES}
)

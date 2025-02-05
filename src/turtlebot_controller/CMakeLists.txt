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

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_square src/turtle_listener.cpp)

target_link_libraries(turtle_square
  ${catkin_LIBRARIES}
)

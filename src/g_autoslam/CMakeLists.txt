cmake_minimum_required(VERSION 2.8.3)
project(g_autoslam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  move_base
  gmapping
  frontier_exploration
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

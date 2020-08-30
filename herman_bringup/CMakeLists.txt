cmake_minimum_required(VERSION 3.0.2)
project(herman_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES herman_bringup
#  CATKIN_DEPENDS ur_bringup ur_modern_driver urg_node
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

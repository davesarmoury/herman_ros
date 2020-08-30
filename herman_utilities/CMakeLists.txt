cmake_minimum_required(VERSION 3.0.2)
project(herman_utilities)

find_package(catkin REQUIRED laser_geometry tf roscpp sensor_msgs)

catkin_package(
  CATKIN_DEPENDS laser_geometry tf roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
target_link_libraries(scan_to_cloud ${catkin_LIBRARIES})


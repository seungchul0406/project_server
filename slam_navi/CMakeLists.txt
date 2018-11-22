cmake_minimum_required(VERSION 2.8.3)
project(slam_navi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs  
)

install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
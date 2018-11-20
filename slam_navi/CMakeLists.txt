cmake_minimum_required(VERSION 2.8.3)
project(slam_navi)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  amcl
  map_server
  move_base
  visualization_msgs
  message_generation
)


add_message_files(
  FILES
  PointArray.msg
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package( 

CATKIN_DEPENDS message_runtime
)


install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(RRT src/RRT.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(RRT ${catkin_LIBRARIES})

add_executable(RRT2 src/RRT2.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(RRT2 ${catkin_LIBRARIES})
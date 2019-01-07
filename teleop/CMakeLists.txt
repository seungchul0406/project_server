cmake_minimum_required(VERSION 2.8.3)
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  joy
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy geometry_msgs joy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/teleop_key.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

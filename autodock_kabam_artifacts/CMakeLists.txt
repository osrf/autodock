cmake_minimum_required(VERSION 3.0.2)
project(autodock_kabam)

## Find catkin dependencies
find_package(catkin REQUIRED)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS message_runtime rospy)

catkin_install_python(PROGRAMS 
  scripts/camera_info_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

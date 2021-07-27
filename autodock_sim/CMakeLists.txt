cmake_minimum_required(VERSION 2.8.3)
project(autodock_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  cmake_modules
  roslib
  roscpp
  geometry_msgs
  std_srvs
  tf
  rosgraph_msgs
  dynamic_reconfigure
  std_msgs
  gazebo_msgs
)

catkin_package(
  LIBRARIES
    gazebo_ros_api_plugin
    gazebo_ros_paths_plugin

  CATKIN_DEPENDS
    roslib
    roscpp
    geometry_msgs
    std_srvs
    tf
    rosgraph_msgs
    dynamic_reconfigure
    std_msgs
    gazebo_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# build charging station plugin
add_library(ChargingStationPlugin SHARED src/ChargingStationPlugin.cpp)
target_link_libraries(
  ChargingStationPlugin
  FlashLightPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES})
  
# Install Gazebo launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

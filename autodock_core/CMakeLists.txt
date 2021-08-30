cmake_minimum_required(VERSION 3.0.2)
project(autodock_core)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS actionlib message_generation)

catkin_python_setup()

## Add actions
add_action_files(DIRECTORY action FILES AutoDocking.action)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp actionlib
)

catkin_install_python(PROGRAMS 
  scripts/simple_autodock.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


## autodock obstacle observer
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(obstacle_observer src/ObstacleObserver.cpp)
target_link_libraries(obstacle_observer ${catkin_LIBRARIES})
add_dependencies( obstacle_observer
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS obstacle_observer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
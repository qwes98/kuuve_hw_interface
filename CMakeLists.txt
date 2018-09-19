cmake_minimum_required(VERSION 2.8.3)
project(kuuvef1_hw_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  serial
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    #kuuvef1cpp
    steer_drive_controller
    serial
  LIBRARIES 
    kuuvef1_hw_interface
    #kuuvef1cpp
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(kuuvef1_hw_interface src/kuuvef1_hw_interface.cpp)
add_library(kuuvef1_motor src/kuuvef1_motor.cpp)
#add_dependencies(kuuvef1_hw_interface kuuvef1cpp)
target_link_libraries(kuuvef1_hw_interface
	kuuvef1_motor
#	${kuuvef1cpp_LIBRARIES}
	${catkin_LIBRARIES})

add_executable(kuuvef1_hw_interface_node src/kuuvef1_hw_interface_node.cpp)
add_dependencies(kuuvef1_hw_interface_node kuuvef1_hw_interface)

target_link_libraries(kuuvef1_hw_interface_node
	kuuvef1_motor
	kuuvef1_hw_interface
	${catkin_LIBRARIES})

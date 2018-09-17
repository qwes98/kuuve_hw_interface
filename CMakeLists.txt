cmake_minimum_required(VERSION 2.8.3)
project(kuuve_hw_interface)

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
	kuuvef1cpp
  LIBRARIES 
    kuuve_hw_interface
	kuuvef1cpp
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

# FIXME: rename cpp files
add_library(kuuve_hw_interface src/tr1_hardware_interface.cpp)
add_dependencies(kuuve_hw_interface kuuvef1cpp)
target_link_libraries(kuuve_hw_interface
	${kuuvef1cpp_LIBRARIES}
	${catkin_LIBRARIES})

add_executable(kuuve_hw_interface_node src/tr1_hardware_interface_node.cpp)
add_dependencies(kuuve_hw_interface_node kuuve_hw_interface)

target_link_libraries(kuuve_hw_interface_node
	kuuve_hw_interface
	${catkin_LIBRARIES})

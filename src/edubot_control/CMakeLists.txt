cmake_minimum_required(VERSION 3.0.2)
project(edubot_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_state_controller
  robot_state_publisher
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager hardware_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_socket
    src/robot_socket.cpp
)

add_library(edubot_hardware_interfaces 
    src/edubot_hw.cpp
)

add_executable(edubot_control_node
    src/edubot_control_node.cpp
)
target_link_libraries(edubot_control_node edubot_hardware_interfaces robot_socket ${catkin_LIBRARIES})
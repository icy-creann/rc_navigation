cmake_minimum_required(VERSION 3.0.2)
project(usb_can_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  can_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy serial std_msgs can_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_can_node src/serial_can_node.cpp)
target_link_libraries(serial_can_node ${catkin_LIBRARIES})

add_executable(serial_can_recv_node src/serial_can_recv_node.cpp)
target_link_libraries(serial_can_recv_node ${catkin_LIBRARIES})

add_executable(serial_can_send_node src/serial_can_send_node.cpp)
target_link_libraries(serial_can_send_node ${catkin_LIBRARIES})
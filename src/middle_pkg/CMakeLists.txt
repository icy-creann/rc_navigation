cmake_minimum_required(VERSION 3.0.2)
project(middle_pkg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  interface_pkg
  move_base_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  interface_pkg
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES middle_pkg
  CATKIN_DEPENDS 
  actionlib
  geometry_msgs
  interface_pkg
  move_base_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
#  DEPENDS system_lib
)

add_definitions(-DPTHREAD_STACK_MIN=0)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(middle_node src/middle_node.cpp)
target_link_libraries(middle_node
  ${catkin_LIBRARIES}
)

add_executable(middle_without_decision src/middle_without_decision.cpp)
target_link_libraries(middle_without_decision
  ${catkin_LIBRARIES}
)

add_executable(can_node src/can_node.cpp)
target_link_libraries(can_node
  ${catkin_LIBRARIES}
)

add_executable(can_sub_node src/can_sub_node.cpp)
target_link_libraries(can_sub_node
  ${catkin_LIBRARIES}
)
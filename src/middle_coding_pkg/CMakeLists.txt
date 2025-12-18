cmake_minimum_required(VERSION 3.0.2)
project(middle_coding_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  interface_pkg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES middle_coding_pkg
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs interface_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(middle_coding_node src/middle_coding_node.cpp)
target_link_libraries(middle_coding_node ${catkin_LIBRARIES})
add_dependencies(middle_coding_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(serial_test_node src/serial_test.cpp)
target_link_libraries(serial_test_node ${catkin_LIBRARIES})
add_dependencies(serial_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
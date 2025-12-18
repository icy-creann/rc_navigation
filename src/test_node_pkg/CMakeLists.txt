cmake_minimum_required(VERSION 3.0.2)
project(test_node_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interface_pkg
  message_runtime
  geometry_msgs
  can_msgs
  roscpp
  std_msgs
  visualization_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_node_pkg
#  CATKIN_DEPENDS interface_pkg message_runtime roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(decision_test_node src/test_node.cpp)

add_dependencies(decision_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(decision_test_node
  ${catkin_LIBRARIES}
)

add_executable(test_node src/test_node_serial.cpp)
add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

add_executable(my_navigator_node src/my_navigator_node.cpp)
add_dependencies(my_navigator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_navigator_node
  ${catkin_LIBRARIES}
)

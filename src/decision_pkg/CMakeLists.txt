cmake_minimum_required(VERSION 3.0.2)
project(decision_pkg)

find_package(catkin REQUIRED COMPONENTS
  interface_pkg
  message_runtime
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES decision_pkg
   CATKIN_DEPENDS interface_pkg message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(decision_node "src/decision_node.cpp")
target_link_libraries(decision_node ${catkin_LIBRARIES} )
add_dependencies(decision_node interface_pkg_generate_messages_cpp)

catkin_install_python(PROGRAMS scripts/topic_relayer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# 通信接口定义
使用方式：   
1、在本包的 package.xml 中添加：
```xml

<build_depend>std_msgs</build_depend>
<build_depend>message_generation</build_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>message_runtime</exec_depend>
```
2、在本包的 CMakeLists.txt 中添加：
```cpp
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  pose.msg
  position.msg
  target.msg
  toSTM32.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)
```

3、在需要调用的包的 package.xml 中添加：
```xml
<build_depend>interface_pkg</build_depend>
<exec_depend>interface_pkg</exec_depend>
```

4、在需要调用的包的 CMakeLists.txt 中添加：
```cpp
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  interface_pkg
)
```

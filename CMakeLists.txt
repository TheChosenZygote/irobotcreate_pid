cmake_minimum_required(VERSION 2.8.3)
project(irobotcreate_pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid_control_node src/pid_control_node.cpp)
add_executable(pid_control_node2 src/pid_control_node2.cpp)
add_executable(pid_control_node3 src/pid_control_node3.cpp)
target_link_libraries(pid_control_node ${catkin_LIBRARIES})
target_link_libraries(pid_control_node2 ${catkin_LIBRARIES})
target_link_libraries(pid_control_node3 ${catkin_LIBRARIES})

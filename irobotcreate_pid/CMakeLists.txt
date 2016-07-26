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

add_executable(irobotcreate_pid_node irobotcreate_pid_node.cpp)
target_link_libraries(irobotcreate_pid_node ${catkin_LIBRARIES})

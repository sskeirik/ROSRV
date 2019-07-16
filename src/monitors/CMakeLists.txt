cmake_minimum_required(VERSION 2.8.3)
project(rvmonitor_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package()

# add_executable(monitor_node src/rvmonitor.cpp)
# target_link_libraries(monitor_node ${catkin_LIBRARIES})


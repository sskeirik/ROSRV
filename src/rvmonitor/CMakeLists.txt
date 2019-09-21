cmake_minimum_required(VERSION 2.8.3)
project(rvmonitor)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

find_package(catkin REQUIRED COMPONENTS roscpp)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rvmonitor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( include )

## Declare a C++ library
add_library(librvmonitor
  src/monitor.cpp
)
target_include_directories(librvmonitor PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(librvmonitor ${catkin_LIBRARIES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## monitor

function(build_monitor spec_file monitor_name)
  set(output_prefix ${PROJECT_BINARY_DIR}/monitors/${monitor_name})
  add_custom_command(
      OUTPUT ${output_prefix}.cpp ${output_prefix}.h
      COMMAND mkdir -p ${PROJECT_BINARY_DIR}/monitors/ &&
	      rosmop --output-prefix ${output_prefix}
		     --monitor-as-node
		     ${spec_file}
      DEPENDS ${spec_file}
  )
  add_executable(${monitor_name} ${output_prefix}.cpp)
  target_include_directories(${monitor_name} PRIVATE ${PROJECT_BINARY_DIR}/monitors/)
  target_link_libraries(${monitor_name} librvmonitor)
endfunction()

build_monitor(${PROJECT_SOURCE_DIR}/specs/single-parameter.rv monitor-single-parameter)
build_monitor(${PROJECT_SOURCE_DIR}/specs/multiple-parameters.rv monitor-multiple-parameters)
build_monitor(${PROJECT_SOURCE_DIR}/specs/multiple-channels.rv monitor-multiple-channels)
build_monitor(${PROJECT_SOURCE_DIR}/specs/dl-watertank.rv monitor-dl-watertank)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rvmonitor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

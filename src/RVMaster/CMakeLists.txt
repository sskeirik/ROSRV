cmake_minimum_required(VERSION 2.8.3)
project(rvmaster)
set(CMAKE_CXX_STANDARD 11)

## ROS Master

LIST(APPEND CMAKE_PROGRAM_PATH  "./rosmop/bin" ...)
add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/monitor/rvmonitor.cpp
           ${PROJECT_BINARY_DIR}/monitor/rvmonitor.h
    COMMAND mkdir -p ${PROJECT_BINARY_DIR}/monitor/ &&
            rosmop --output-prefix ${PROJECT_BINARY_DIR}/monitor/rvmonitor
                   ${PROJECT_SOURCE_DIR}/rvmonitor.rv
    DEPENDS rvmonitor.rv
)
include_directories(${PROJECT_BINARY_DIR}/monitor/)

get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} PATH)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)
find_package(Boost REQUIRED COMPONENTS regex program_options)
catkin_package( INCLUDE_DIRS include )

include_directories(include/ ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )

add_library( librvmaster
             ${PROJECT_BINARY_DIR}/monitor/rvmonitor.cpp
             src/xmlrpcpp/XmlRpcClient.cpp
             src/xmlrpcpp/XmlRpcServerConnection.cpp
             src/xmlrpcpp/XmlRpcServerMethod.cpp
             src/xmlrpcpp/XmlRpcSocket.cpp
             src/xmlrpcpp/XmlRpcServer.cpp
             src/xmlrpcpp/XmlRpcSource.cpp
             src/xmlrpcpp/XmlRpcDispatch.cpp
             src/rv/xmlrpc_manager.cpp
             src/rv/connection_manager.cpp
             src/rv/transport_publisher_link.cpp
             src/rv/transport_subscriber_link.cpp
             src/rv/subscription.cpp
             src/rv/publisher_link.cpp
             src/rv/server_manager.cpp
             src/rv/master.cpp
             src/rv/acctrl_manager.cpp
             src/rv/monitor.cpp
           )
target_link_libraries(librvmaster ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(rvmaster src/main.cpp)
target_link_libraries(rvmaster librvmaster)

## Isolated monitor

add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/isolated-monitor/rvmonitor.cpp
           ${PROJECT_BINARY_DIR}/isolated-monitor/rvmonitor.h
    COMMAND mkdir -p ${PROJECT_BINARY_DIR}/isolated-monitor/ &&
            rosmop --output-prefix ${PROJECT_BINARY_DIR}/isolated-monitor/rvmonitor
                   --monitor-as-node
                   ${PROJECT_SOURCE_DIR}/rvmonitor.rv
    DEPENDS rvmonitor.rv
)
add_executable(test-isolated-monitor ${PROJECT_BINARY_DIR}/isolated-monitor/rvmonitor.cpp)
target_include_directories(test-isolated-monitor PRIVATE ${PROJECT_BINARY_DIR}/isolated-monitor/)
target_link_libraries(test-isolated-monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Install

install( TARGETS rvmaster
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install( PROGRAMS ${PARENT_DIR}/bin/${PROJECT_NAME}.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install( DIRECTORY ${PARENT_DIR}/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE
  PATTERN ".*.swp" EXCLUDE
  )

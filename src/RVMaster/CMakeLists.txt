cmake_minimum_required(VERSION 2.8.3)
project(rvmaster)
set(CMAKE_CXX_STANDARD 11)

## ROS Master

get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} PATH)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)
catkin_package( INCLUDE_DIRS include
                DEPENDS rvmonitor
              )
find_package(Boost REQUIRED COMPONENTS regex program_options)

include_directories(include/ ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )

add_library( librvmaster
             src/xmlrpcpp/XmlRpcClient.cpp
             src/xmlrpcpp/XmlRpcServerConnection.cpp
             src/xmlrpcpp/XmlRpcServerMethod.cpp
             src/xmlrpcpp/XmlRpcSocket.cpp
             src/xmlrpcpp/XmlRpcServer.cpp
             src/xmlrpcpp/XmlRpcSource.cpp
             src/xmlrpcpp/XmlRpcDispatch.cpp
             src/rv/xmlrpc_manager.cpp
             src/rv/server_manager.cpp
             src/rv/master.cpp
             src/rv/acctrl_manager.cpp
           )
target_link_libraries(librvmaster ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(rvmaster src/main.cpp)
target_link_libraries(rvmaster librvmaster)

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

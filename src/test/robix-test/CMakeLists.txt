cmake_minimum_required(VERSION 2.8.3)
project(robix-test)

find_package(catkin REQUIRED)

find_package(marti_common_msgs REQUIRED)

catkin_package()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   robix-test.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

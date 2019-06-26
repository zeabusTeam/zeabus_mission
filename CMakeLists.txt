cmake_minimum_required(VERSION 2.8.3)
project(zeabus_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zeabus_mission
#  CATKIN_DEPENDS message_generation rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set( PYTHON_CODE 
    src/square_2.py
    src/gate_lib.py
)

catkin_install_python(
    PROGRAMS ${PYTHON_CODE}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

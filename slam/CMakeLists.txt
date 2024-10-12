cmake_minimum_required(VERSION 3.0.2)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slam
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/EKFO_landmarks.py
  scripts/test_wrench.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
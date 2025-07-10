cmake_minimum_required(VERSION 3.0.2)
project(control)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
  std_msgs 
  sensor_msgs
  geometry_msgs 
  nav_msgs
  message_generation
)

catkin_python_setup()

find_package(Eigen3 REQUIRED)

##################################
## Declare ROS messages/services ##
##################################

add_service_files(
  FILES
  SetDesired.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hosmc_controller
  CATKIN_DEPENDS rospy roscpp sensor_msgs std_msgs geometry_msgs nav_msgs message_runtime
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_lib
  CATKIN_DEPENDS rospy roscpp sensor_msgs std_msgs geometry_msgs nav_msgs message_runtime
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(control_lib
  src/PathTracker.cpp
)

add_library(hosmc_controller
  src/hosmc_controller.cpp
)


add_dependencies(hosmc_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hosmc_controller
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(control_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(HOSMC_node_cpp nodes/hosmc_node_cpp.cpp)
add_executable(path_tracker_node nodes/path_tracker_node.cpp)
add_dependencies(HOSMC_node_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(HOSMC_node_cpp
  hosmc_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(path_tracker_node
  control_lib
  ${catkin_LIBRARIES}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  nodes/hosmc_node_py.py
  nodes/mpc_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package()

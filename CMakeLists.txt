cmake_minimum_required(VERSION 2.8.3)
project(ros_orchestrator)

find_package(catkin REQUIRED rospy roslaunch genmsg std_msgs)

catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Orchestrator.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories()


catkin_install_python(PROGRAMS src/orchestrator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

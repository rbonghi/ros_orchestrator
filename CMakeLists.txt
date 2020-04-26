cmake_minimum_required(VERSION 2.8.3)
project(ros_orchestrator)

find_package(catkin REQUIRED rospy roslaunch)

catkin_python_setup()

catkin_package()

include_directories()

catkin_install_python(PROGRAMS src/orchestrator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

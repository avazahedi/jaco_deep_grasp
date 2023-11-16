cmake_minimum_required(VERSION 3.1.3)
project(jaco_grasp_ros)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_task_constructor_demo
	moveit_task_constructor_gpd
	rospy
	sensor_msgs
	control_msgs
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp
)

catkin_install_python(PROGRAMS src/traj_plot.py src/object_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

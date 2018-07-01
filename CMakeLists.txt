cmake_minimum_required(VERSION 2.8.3)
project(face)

find_package(catkin REQUIRED COMPONENTS 
	roscpp rospy std_msgs sensor_msgs 
	genmsg geometry_msgs visualization_msgs
	nodelet tf image_geometry cv_bridge  camera_info_manager)

add_message_files(FILES vector.msg)
add_service_files(FILES center.srv)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS sensor_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime)

install(PROGRAMS 
	include/face.py 
	src/lee.py 
	src/escribe.py 
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
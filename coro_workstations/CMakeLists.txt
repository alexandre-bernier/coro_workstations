cmake_minimum_required(VERSION 3.1.3)
project(coro_workstations)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    rviz_visual_tools)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	moveit_core
	moveit_ros_planning_interface
	moveit_visual_tools)
	
include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_workstation nodes/test_workstation.cpp)
target_link_libraries(test_workstation ${catkin_LIBRARIES})

install(PROGRAMS scripts/copy_file.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_workstation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

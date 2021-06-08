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
	robotiq_85_msgs
	robotiq_ft_sensor)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	moveit_core
	moveit_ros_planning_interface)
	
include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_sim_workstation nodes/test_sim_workstation.cpp)
target_link_libraries(test_sim_workstation ${catkin_LIBRARIES})
install(TARGETS test_sim_workstation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_hw_workstation nodes/test_hw_workstation.cpp)
target_link_libraries(test_hw_workstation ${catkin_LIBRARIES})
install(TARGETS test_hw_workstation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

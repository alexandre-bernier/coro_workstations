/**
 * @brief Code example for Moveit. Makes the robot do a little mouvement around its current position.
 * @warning Make sure the robot can move freely before launching this node.
 * @author Alexandre Bernier
 * @date May 12th, 2021
 * @copyright BSD
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
	// ROS init
	ros::init(argc, argv, "test_workstation");
	ros::NodeHandle node_handle;
	
	// ROS spinning must be running for the MoveGroupInterface to get information
	// about the robot's state. One way to do this is to start an AsyncSpinner
	// beforehand.
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// The name of the planning group can be found in the .srdf of the workstation's moveit_config directory.
	// (The planning groups should be named "arm" for the robot and "gripper" for the end-effector.)
	static const std::string ARM_PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface arm_move_group_interface(ARM_PLANNING_GROUP);
	
	// Raw pointers are frequently used to refer to the planning group for improved performance.
	const moveit::core::JointModelGroup* arm_joint_model_group = arm_move_group_interface.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
	
	// Basic information
	ROS_INFO_NAMED("test_workstation", "Planning frame: %s", arm_move_group_interface.getPlanningFrame().c_str());
	ROS_INFO_NAMED("test_workstation", "End effector link: %s", arm_move_group_interface.getEndEffectorLink().c_str());
	
	return 0;
}

/**
 * @brief Code example for Moveit. Makes the robot do a little mouvement around its current position.
 * @warning Make sure the robot can move freely before launching this node.
 * @author Alexandre Bernier
 * @date May 12th, 2021
 * @copyright BSD
 */

#include <math.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// The name of the planning group can be found in the .srdf of the workstation's moveit_config directory
// (The planning groups should be named "arm" for the robot and "gripper" for the end-effector)
static const std::string ARM_PLANNING_GROUP = "arm";
moveit::planning_interface::MoveGroupInterface *arm_move_group_interface;

// TF listener allows us to get transforms between any two frames in the urdf.
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

void move_relative_to_tcp(float x, float y, float z, float rx, float ry, float rz)
{
	// Variables
	tf2::Transform tf_arm_world_to_tcp, tf_tcp_to_goal, tf_world_to_goal;
	geometry_msgs::Pose world_to_goal;
	
	// Current tcp pose
	geometry_msgs::Pose arm_world_to_tcp = arm_move_group_interface->getCurrentPose().pose;
	// *DEBUG*
	ROS_INFO_NAMED("test_workstation", "Current tcp position: (%f : %f : %f)", arm_world_to_tcp.position.x, arm_world_to_tcp.position.y, arm_world_to_tcp.position.z);
	ROS_INFO_NAMED("test_workstation", "Current tcp orientation: (%f : %f : %f)", arm_world_to_tcp.orientation.x, arm_world_to_tcp.orientation.y, arm_world_to_tcp.orientation.z);
	
	// Convert current pose to tf_pose
	tf2::fromMsg(arm_world_to_tcp, tf_arm_world_to_tcp);
	
	// Create the tf_pose "tf_tcp_to_goal"
	tf_tcp_to_goal.setOrigin(tf2::Vector3(x,y,z));
	tf2::Quaternion q;
	q.setRPY(rx,ry,rz);
	tf_tcp_to_goal.setRotation(q);
	// *DEBUG*
	ROS_INFO_NAMED("test_workstation", "Goal tcp offset position: (%f : %f : %f)", tf_tcp_to_goal.getOrigin()[0], tf_tcp_to_goal.getOrigin()[1], tf_tcp_to_goal.getOrigin()[2]);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	ROS_INFO_NAMED("test_workstation", "Goal tcp offset orientation: (%f : %f : %f)", roll, pitch, yaw);
	
	// Get the tf_pose world_to_goal and convert it back to geometry_msgs
	tf_world_to_goal = tf_arm_world_to_tcp * tf_tcp_to_goal;
	tf2::toMsg(tf_world_to_goal, world_to_goal);
	
	// Plan and execute the move
// 	arm_move_group_interface->clearPoseTargets();
	arm_move_group_interface->setPoseTarget(world_to_goal);
	arm_move_group_interface->move();
}

int main(int argc, char** argv)
{
	// ROS init
	ros::init(argc, argv, "test_workstation");
	ros::NodeHandle node_handle;
	
	// ROS spinning must be running for the MoveGroupInterface to get information
	// about the robot's state. One way to do this is to start an AsyncSpinner beforehand
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// Initialize global pointers
	arm_move_group_interface = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	
	// Basic information
	ROS_INFO_NAMED("test_workstation", "Planning frame: %s", arm_move_group_interface->getPlanningFrame().c_str());
	ROS_INFO_NAMED("test_workstation", "Pose reference frame: %s", arm_move_group_interface->getPoseReferenceFrame().c_str());
	ROS_INFO_NAMED("test_workstation", "End effector link: %s", arm_move_group_interface->getEndEffectorLink().c_str());
	
	// Move the TCP by giving an offset from the current position (using "tcp" axes)
	// X
	move_relative_to_tcp(0.05, 0, 0, 0, 0, 0);
	move_relative_to_tcp(-0.1, 0, 0, 0, 0, 0);
	move_relative_to_tcp(0.05, 0, 0, 0, 0, 0);
	//Y
	move_relative_to_tcp(0, 0.05, 0, 0, 0, 0);
	move_relative_to_tcp(0, -0.1, 0, 0, 0, 0);
	move_relative_to_tcp(0, 0.05, 0, 0, 0, 0);
	// Z
	move_relative_to_tcp(0, 0, 0.05, 0, 0, 0);
	move_relative_to_tcp(0, 0, -0.1, 0, 0, 0);
	move_relative_to_tcp(0, 0, 0.05, 0, 0, 0);
	// Rx
	move_relative_to_tcp(0, 0, 0, M_PI/16, 0, 0);
	move_relative_to_tcp(0, 0, 0, -M_PI/8, 0, 0);
	move_relative_to_tcp(0, 0, 0, M_PI/16, 0, 0);
	// Ry
	move_relative_to_tcp(0, 0, 0, 0, M_PI/16, 0);
	move_relative_to_tcp(0, 0, 0, 0, -M_PI/8, 0);
	move_relative_to_tcp(0, 0, 0, 0, M_PI/16, 0);
	// Rz
	move_relative_to_tcp(0, 0, 0, 0, 0, M_PI/16);
	move_relative_to_tcp(0, 0, 0, 0, 0, -M_PI/8);
	move_relative_to_tcp(0, 0, 0, 0, 0, M_PI/16);
	
	// Close the node and exit
	ros::shutdown();
	return 0;
}

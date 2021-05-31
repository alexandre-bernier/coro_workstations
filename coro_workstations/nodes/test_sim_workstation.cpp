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

// Gripper states for the "gripper" function
#define GRIPPER_OPEN (0u)
#define GRIPPER_FULL_CLOSE (1u)
#define GRIPPER_HALF_CLOSE (2u)

// The name of the planning group can be found in the .srdf of the workstation's moveit_config directory
// (The planning groups should be named "arm" for the robot and "gripper" for the end-effector)
static const std::string ARM_PLANNING_GROUP = "arm";
static const std::string GRIPPER_PLANNING_GROUP = "gripper";
moveit::planning_interface::MoveGroupInterface *arm_move_group_interface;
moveit::planning_interface::MoveGroupInterface *gripper_move_group_interface;

std::string gripper_active_joint_name = "";
double closed_gripper_joint_value = 0;

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
	ROS_INFO_NAMED("test_sim_workstation", "Current tcp position: (%f : %f : %f)", arm_world_to_tcp.position.x, arm_world_to_tcp.position.y, arm_world_to_tcp.position.z);
	ROS_INFO_NAMED("test_sim_workstation", "Current tcp orientation: (%f : %f : %f)", arm_world_to_tcp.orientation.x, arm_world_to_tcp.orientation.y, arm_world_to_tcp.orientation.z);
	
	// Convert current pose to tf_pose
	tf2::fromMsg(arm_world_to_tcp, tf_arm_world_to_tcp);
	
	// Create the tf_pose "tf_tcp_to_goal"
	tf_tcp_to_goal.setOrigin(tf2::Vector3(x,y,z));
	tf2::Quaternion q;
	q.setRPY(rx,ry,rz);
	tf_tcp_to_goal.setRotation(q);
	// *DEBUG*
	ROS_INFO_NAMED("test_sim_workstation", "Goal tcp offset position: (%f : %f : %f)", tf_tcp_to_goal.getOrigin()[0], tf_tcp_to_goal.getOrigin()[1], tf_tcp_to_goal.getOrigin()[2]);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	ROS_INFO_NAMED("test_sim_workstation", "Goal tcp offset orientation: (%f : %f : %f)", roll, pitch, yaw);
	
	// Get the tf_pose world_to_goal and convert it back to geometry_msgs
	tf_world_to_goal = tf_arm_world_to_tcp * tf_tcp_to_goal;
	tf2::toMsg(tf_world_to_goal, world_to_goal);
	
	// Plan and execute the move
	arm_move_group_interface->setPoseTarget(world_to_goal);
	arm_move_group_interface->move();
}

void gripper(int req_state)
{
	bool target_set = false;
	
	switch(req_state) {
		case GRIPPER_OPEN:
			target_set = gripper_move_group_interface->setNamedTarget("open");	// One of the only two named targets specified in the srdf
			ROS_INFO_NAMED("test_sim_workstation", "Opening gripper...");
			break;
			
		case GRIPPER_FULL_CLOSE:
			target_set = gripper_move_group_interface->setNamedTarget("close");	// The other named target specified in the srdf
			ROS_INFO_NAMED("test_sim_workstation", "Closing gripper...");
			break;
			
		case GRIPPER_HALF_CLOSE:
			target_set = gripper_move_group_interface->setJointValueTarget(gripper_active_joint_name, 0.5*closed_gripper_joint_value);
			ROS_INFO_NAMED("test_sim_workstation", "Closing/Opening gripper to half...");
			break;
	}
	
	if(target_set)
		gripper_move_group_interface->move();
}

int main(int argc, char** argv)
{
	// ROS init
	ros::init(argc, argv, "test_sim_workstation");
	ros::NodeHandle node_handle;
	
	// ROS spinning must be running for the MoveGroupInterface to get information
	// about the robot's state. One way to do this is to start an AsyncSpinner beforehand
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// Initialize global pointers
	arm_move_group_interface = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);
	gripper_move_group_interface = new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP);
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	
	// Basic information
	ROS_INFO_NAMED("test_sim_workstation", "Arm planning frame: %s", arm_move_group_interface->getPlanningFrame().c_str());
	ROS_INFO_NAMED("test_sim_workstation", "Arm pose reference frame: %s", arm_move_group_interface->getPoseReferenceFrame().c_str());
	ROS_INFO_NAMED("test_sim_workstation", "Arm end effector link: %s", arm_move_group_interface->getEndEffectorLink().c_str());
	ROS_INFO_NAMED("test_sim_workstation", "Gripper planning frame: %s", gripper_move_group_interface->getPlanningFrame().c_str());
	ROS_INFO_NAMED("test_sim_workstation", "Gripper pose reference frame: %s", gripper_move_group_interface->getPoseReferenceFrame().c_str());
	
	// Other useful variables
	gripper_active_joint_name = gripper_move_group_interface->getActiveJoints().at(0);
	closed_gripper_joint_value = gripper_move_group_interface->getNamedTargetValues("close").at(gripper_active_joint_name);
	
	// ------------------------------------------
	// MOVE ROBOT
	// Move the robot by giving a TCP's offset from the current position (using "tcp" axes)
	move_relative_to_tcp(0.05, 0, 0, 0, 0, 0);
	move_relative_to_tcp(-0.1, 0, 0, 0, 0, 0);
	move_relative_to_tcp(0.05, 0, 0, 0, 0, 0);
	
	move_relative_to_tcp(0, 0.05, 0, 0, 0, 0);
	move_relative_to_tcp(0, -0.1, 0, 0, 0, 0);
	move_relative_to_tcp(0, 0.05, 0, 0, 0, 0);

	move_relative_to_tcp(0, 0, 0.05, 0, 0, 0);
	move_relative_to_tcp(0, 0, -0.1, 0, 0, 0);
	move_relative_to_tcp(0, 0, 0.05, 0, 0, 0);
	
	move_relative_to_tcp(0, 0, 0, M_PI/16, 0, 0);
	move_relative_to_tcp(0, 0, 0, -M_PI/8, 0, 0);
	move_relative_to_tcp(0, 0, 0, M_PI/16, 0, 0);

	move_relative_to_tcp(0, 0, 0, 0, M_PI/16, 0);
	move_relative_to_tcp(0, 0, 0, 0, -M_PI/8, 0);
	move_relative_to_tcp(0, 0, 0, 0, M_PI/16, 0);

	move_relative_to_tcp(0, 0, 0, 0, 0, M_PI/16);
	move_relative_to_tcp(0, 0, 0, 0, 0, -M_PI/8);
	move_relative_to_tcp(0, 0, 0, 0, 0, M_PI/16);
	// ------------------------------------------
	
	// ------------------------------------------
	// OPERATE GRIPPER
	gripper(GRIPPER_HALF_CLOSE);
	gripper(GRIPPER_FULL_CLOSE);
	gripper(GRIPPER_OPEN);
	// ------------------------------------------
	
	// Close the node and exit
	ros::shutdown();
	return 0;
}

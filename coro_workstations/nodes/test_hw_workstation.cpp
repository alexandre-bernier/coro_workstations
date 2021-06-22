/**
 * @brief Code example for Moveit. Makes the robot do a little mouvement around its current position.
 * @warning Make sure the robot can move freely before launching this node.
 * @author Alexandre Bernier
 * @date May 31st, 2021
 * @copyright BSD
 */

#include <math.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <robotiq_85_msgs/GripperCmd.h>
#include <robotiq_85_msgs/GripperStat.h>
#include <robotiq_ft_sensor/ft_sensor.h>
#include <robotiq_ft_sensor/sensor_accessor.h>

// Gripper states for the "gripper" function
#define GRIPPER_OPEN (0u)
#define GRIPPER_FULL_CLOSE (1u)
#define GRIPPER_HALF_CLOSE (2u)

// The name of the planning group can be found in the .srdf of the workstation's moveit_config directory
// (The planning groups should be named "arm" for the robot and "gripper" for the end-effector)
static const std::string ARM_PLANNING_GROUP = "arm";
moveit::planning_interface::MoveGroupInterface *arm_move_group_interface;

// Gripper
ros::Subscriber sub_gripper_stat;
static const std::string topic_gripper_stat = "/gripper/stat";
robotiq_85_msgs::GripperStat gripper_stat;
static bool gripper_stat_received = false;
ros::Publisher pub_gripper_cmd;
static const std::string topic_gripper_cmd = "/gripper/cmd";

// FT sensor (CB2 and CB3+eSeries have different topics and work in different ways)
ros::Subscriber sub_ft_sensor; 	// CB2 only
ros::Subscriber sub_ft_wrench; 	// CB2 only
ros::Subscriber sub_wrench; 	// CB3+eSeries only
static const std::string topic_ft_sensor = "/robotiq_ft_sensor";	// CB2 only: Custom message from the package "robotiq_ft_sensor"
static const std::string topic_ft_wrench = "/robotiq_ft_wrench";	// CB2 only: Same information than "ft_sensor", but in a ROS standard message
static const std::string topic_wrench = "/wrench";					// CB3+eSeries only: FT sensor data in a ROS standard message
robotiq_ft_sensor::ft_sensor ft_sensor; 	// CB2 only
geometry_msgs::WrenchStamped ft_wrench; 	// CB2 only
geometry_msgs::WrenchStamped wrench; 		// CB3+eSeries only
ros::ServiceClient ft_service_client; 		// CB2 only
static const std::string ft_service_accessor = "robotiq_ft_sensor_acc"; 	// CB2 only

// TF listener allows us to get transforms between any two frames in the urdf.
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

// This function offsets the current pose of the TCP (and commands the robot to move)
void move_relative_to_tcp(float x, float y, float z, float rx, float ry, float rz)
{
	// Variables
	tf2::Transform tf_arm_world_to_tcp, tf_tcp_to_goal, tf_world_to_goal;
	geometry_msgs::Pose world_to_goal;
	
	// Current tcp pose
	geometry_msgs::Pose arm_world_to_tcp = arm_move_group_interface->getCurrentPose().pose;
	// *DEBUG*
	ROS_INFO_NAMED("test_hw_workstation", "Current tcp position: (%f : %f : %f)", arm_world_to_tcp.position.x, arm_world_to_tcp.position.y, arm_world_to_tcp.position.z);
	ROS_INFO_NAMED("test_hw_workstation", "Current tcp orientation: (%f : %f : %f)", arm_world_to_tcp.orientation.x, arm_world_to_tcp.orientation.y, arm_world_to_tcp.orientation.z);
	
	// Convert current pose to tf_pose
	tf2::fromMsg(arm_world_to_tcp, tf_arm_world_to_tcp);
	
	// Create the tf_pose "tf_tcp_to_goal"
	tf_tcp_to_goal.setOrigin(tf2::Vector3(x,y,z));
	tf2::Quaternion q;
	q.setRPY(rx,ry,rz);
	tf_tcp_to_goal.setRotation(q);
	// *DEBUG*
	ROS_INFO_NAMED("test_hw_workstation", "Goal tcp offset position: (%f : %f : %f)", tf_tcp_to_goal.getOrigin()[0], tf_tcp_to_goal.getOrigin()[1], tf_tcp_to_goal.getOrigin()[2]);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	ROS_INFO_NAMED("test_hw_workstation", "Goal tcp offset orientation: (%f : %f : %f)", roll, pitch, yaw);
	
	// Get the tf_pose world_to_goal and convert it back to geometry_msgs
	tf_world_to_goal = tf_arm_world_to_tcp * tf_tcp_to_goal;
	tf2::toMsg(tf_world_to_goal, world_to_goal);
	
	// Plan and execute the move
	arm_move_group_interface->setPoseTarget(world_to_goal);
	arm_move_group_interface->move();
}

void gripper_stat_callback(const robotiq_85_msgs::GripperStatConstPtr& stat)
{
	gripper_stat_received = true;
	gripper_stat = *stat;
	// *DEBUG*
// 	ROS_INFO_NAMED("test_hw_workstation", "gripper_stat.position = %f", gripper_stat.position);
}

// Gripper commands limits:
//	position = [0; 0.085] (0 = closed, 0.085 = opened)
// 	speed = [0.013, 0.1]
// 	force = [5.0, 220.0]
void gripper(int req_state)
{
	robotiq_85_msgs::GripperCmd cmd;
	
	// Set the GripperCmd variable
	switch(req_state) {
		case GRIPPER_OPEN:
			cmd.position = 0.085;
			cmd.speed = 0.05;
			cmd.force = 100;
			ROS_INFO_NAMED("test_hw_workstation", "Opening gripper...");
			break;
			
		case GRIPPER_FULL_CLOSE:
			cmd.position = 0;
			cmd.speed = 0.05;
			cmd.force = 100;
			ROS_INFO_NAMED("test_hw_workstation", "Closing gripper...");
			break;
			
		case GRIPPER_HALF_CLOSE:
			cmd.position = 0.085/2;
			cmd.speed = 0.05;
			cmd.force = 100;
			ROS_INFO_NAMED("test_hw_workstation", "Closing/Opening gripper to half...");
			break;
	}
	
	// Publish the command
	pub_gripper_cmd.publish(cmd);
	
	// Set this flag manually because the next gripper state takes too long to come in
	gripper_stat.is_moving = true;
	
	// Wait for the gripper to stop moving
	while(gripper_stat.is_moving)
		ros::spinOnce();
}

// *** CB2 only ***
// The force-torque sensor publishes 2 messages: 1 custom named "ft_sensor" and one from ROS geometry_msgs named "WrenchStamped"
// Both messages contain the same sensor data. You should subscribe to only one of the two.
// Here, we subscribed to both messages to show how to do it.
void ft_sensor_callback(const robotiq_ft_sensor::ft_sensorConstPtr &ft)
{
	ft_sensor = *ft;
	// *DEBUG*
// 	ROS_INFO_NAMED("test_hw_workstation", "Force-Torque Sensor:\n\tFx = %f\n\tFy = %f\n\tFz = %f\n\tMx = %f\n\tMy = %f\n\tMz = %f",
// 		ft_sensor.Fx, ft_sensor.Fy, ft_sensor.Fz, ft_sensor.Mx, ft_sensor.My, ft_sensor.Mz);
}

// *** CB2 only ***
// The force-torque sensor publishes 2 messages: 1 custom named "ft_sensor" and one from ROS geometry_msgs named "WrenchStamped"
// Both messages contain the same sensor data. You should subscribe to only one of the two.
// Here, we subscribed to both messages to show how to do it.
void ft_wrench_callback(const geometry_msgs::WrenchStampedConstPtr &ft)
{
	ft_wrench = *ft;
	// *DEBUG*
// 	ROS_INFO_NAMED("test_hw_workstation", "Force-Torque Wrench:\n\tFx = %f\n\tFy = %f\n\tFz = %f\n\tMx = %f\n\tMy = %f\n\tMz = %f",
// 				   ft_wrench.wrench.force.x, ft_wrench.wrench.force.y, ft_wrench.wrench.force.z, ft_wrench.wrench.torque.x, ft_wrench.wrench.torque.y, ft_wrench.wrench.torque.z);
}

// *** CB3+eSeries only ***
// The force-torque sensor data is published by the hardware_interface node under the topic "/wrench".
// The force-torque sensor will zero itself once at the launch of the workstation (launch file).
// You can't request the force-torque sensor to zero itself afterwards. You'll have to do it programmically if you need to.
void wrench_callback(const geometry_msgs::WrenchStampedConstPtr &ft)
{
	wrench = *ft;
	// *DEBUG*
// 		ROS_INFO_NAMED("test_hw_workstation", "Force-Torque Wrench:\n\tFx = %f\n\tFy = %f\n\tFz = %f\n\tMx = %f\n\tMy = %f\n\tMz = %f",
// 					   wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z, wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z);
}

int main(int argc, char** argv)
{
	// ROS init
	ros::init(argc, argv, "test_hw_workstation");
	ros::NodeHandle node_handle;
	
	// ROS spinning must be running for the MoveGroupInterface to get information
	// about the robot's state. One way to do this is to start an AsyncSpinner beforehand
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// Initialize global pointers
	// MoveIt
	arm_move_group_interface = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);
	// Gripper
	sub_gripper_stat = node_handle.subscribe(topic_gripper_stat, 10, gripper_stat_callback);
	pub_gripper_cmd = node_handle.advertise<robotiq_85_msgs::GripperCmd>(topic_gripper_cmd, 10);
	// Force-Torque Sensor
	sub_ft_sensor = node_handle.subscribe(topic_ft_sensor, 1000, ft_sensor_callback); 	// CB2 only
	sub_ft_wrench = node_handle.subscribe(topic_ft_wrench, 1000, ft_wrench_callback); 	// CB2 only
	sub_wrench = node_handle.subscribe(topic_wrench, 1000, wrench_callback); 			// CB3+eSeries only
	ft_service_client = node_handle.serviceClient<robotiq_ft_sensor::sensor_accessor>(ft_service_accessor); 	// CB2 only
	robotiq_ft_sensor::sensor_accessor ft_srv; 	// CB2 only
	// TF
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	
	// Basic information
	ROS_INFO_NAMED("test_hw_workstation", "Arm planning frame: %s", arm_move_group_interface->getPlanningFrame().c_str());
	ROS_INFO_NAMED("test_hw_workstation", "Arm pose reference frame: %s", arm_move_group_interface->getPoseReferenceFrame().c_str());
	ROS_INFO_NAMED("test_hw_workstation", "Arm end effector link: %s", arm_move_group_interface->getEndEffectorLink().c_str());
	
	// ------------------------------------------
	// CB2 only
	// Zero Force-Torque Sensor
	// Make sure no forces are applied before zeroing the sensor
	ft_srv.request.command_id = ft_srv.request.COMMAND_SET_ZERO;
	ft_service_client.call(ft_srv);
	ROS_INFO_NAMED("test_hw_workstation", "Force-Torque Sensor zero request: %s", ft_srv.response.res.c_str());
	// ------------------------------------------
	
	// Wait for first gripper state to come in
	while(!gripper_stat_received)
		ros::spinOnce();
		
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
	ros::Duration(1).sleep();	// Adding a sleep just so we can see a pause between each movement
	gripper(GRIPPER_FULL_CLOSE);
	ros::Duration(1).sleep();	// Adding a sleep just so we can see a pause between each movement
	gripper(GRIPPER_HALF_CLOSE);
	ros::Duration(1).sleep();	// Adding a sleep just so we can see a pause between each movement
	gripper(GRIPPER_OPEN);
	ros::Duration(1).sleep();	// Adding a sleep just so we can see a pause between each movement
	// ------------------------------------------
	
	// Close the node and exit
	ros::shutdown();
	return 0;
}

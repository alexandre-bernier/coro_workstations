#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class TestMoveRobot:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('test_move_robot', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Move Group: arm
        self.arm_group_name = "arm"
        self.arm_move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.arm_move_group.set_pose_reference_frame("base_link")

        # Move Group: gripper
        self.gripper_group_name = "gripper"
        self.gripper_move_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)
        self.gripper_move_group.set_pose_reference_frame("tool0")

        # Rviz trajectory visualization
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # Basic information
        self.arm_planning_frame = self.arm_move_group.get_planning_frame()
        print("============ Arm planning frame: %s" % self.arm_planning_frame)
        self.arm_pose_reference_frame = self.arm_move_group.get_pose_reference_frame()
        print("============ Arm pose reference frame: %s" % self.arm_pose_reference_frame)
        self.gripper_planning_frame = self.gripper_move_group.get_planning_frame()
        print("============ Gripper planning frame: %s" % self.gripper_planning_frame)
        self.gripper_pose_reference_frame = self.gripper_move_group.get_pose_reference_frame()
        print("============ Gripper pose reference frame: %s" % self.gripper_pose_reference_frame)
        self.eef_link = self.arm_move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def plan_and_execute_trajectory(self):
        delta = 0.01
        waypoints = []
        wpose = geometry_msgs.msg.PoseStamped()
        wpose.header.frame_id = self.eef_link

        # Waypoint 1: X-
        wpose.pose.position.x = delta
        waypoints.append(copy.deepcopy(wpose))
        # Waypoint 2: X+
        # wpose.position.x += 2*delta
        # waypoints.append(copy.deepcopy(wpose))
        # # Waypoint 3: Y-
        # wpose = self.arm_move_group.get_current_pose().pose
        # wpose.position.y -= delta
        # waypoints.append(copy.deepcopy(wpose))
        # # Waypoint 4: Y+
        # wpose.position.y += 2*delta
        # waypoints.append(copy.deepcopy(wpose))
        # # Waypoint 5: Z-
        # wpose = self.arm_move_group.get_current_pose().pose
        # wpose.position.z -= delta
        # waypoints.append(copy.deepcopy(wpose))
        # # Waypoint 6: Z+
        # wpose.position.z += 2*delta
        # waypoints.append(copy.deepcopy(wpose))

        # Plan trajectory
        self.arm_move_group.set_pose_target(wpose)
        success, trajectory, planning_time, errors = self.arm_move_group.plan()
        print("Planning success: %s" % success)
        print("Trajectory: %s" % trajectory)
        print("Planning time: %s" % planning_time)
        print("Errors: %s" % errors)
        if success:
            self.arm_move_group.execute(trajectory)
        self.arm_move_group.clear_pose_targets()


if __name__ == '__main__':
    test = TestMoveRobot()
    test.plan_and_execute_trajectory()
    print("")
    print("Done!")

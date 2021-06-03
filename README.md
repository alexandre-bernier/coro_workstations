# coro_workstations

## Overview

This ROS package is meant as an easy way to use some of the robotics workstation available at the CoRo lab (ÉTS, Montréal). The idea is to create simple launch files that allows the user to either work in simulation or connect directly to the workstations and be able to run (almost) the same code in both cases.

The following workstations are available within this package:

![UR5 Workstation](doc/ur5_workstation.png)

### License

The source code is released under a [BSD 3-Clause license](coro_workstations/LICENSE).

**Author: Alexandre Bernier<br />
Affiliation: [CoRo, ÉTS](http://en.etsmtl.ca/unites-de-recherche/coro/accueil?lang=en-CA)<br />
Maintainer: Alexandre Bernier, ab.alexandre.bernier@gmail.com**

The coro_workstations package has been tested under ROS Noetic on Ubuntu 20.04.

## Installation

### Building from Source

#### Dependencies

The following packages can be installed as Debian packages. See links for installation instructions:

- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (Ubuntu 20.04 is required)
- [MoveIt](https://moveit.ros.org/install/)

You will need to clone and build from source the following packages before trying to build coro_workstations (pay attention to the versions/branches indicated for each package):

- [fmauch/universal_robot](https://github.com/fmauch/universal_robot.git) (calibration-devel branch)
- [ros-industrial/ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver.git) (kinetic-devel branch)
- [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq.git) (kinetic-devel branch)
- [ros-industrial/ur_msgs](https://github.com/ros-industrial/ur_msgs.git) (version 1.3.0)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/alexandre-bernier/coro_workstations.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

Run a simulated workstation with:

	roslaunch coro_workstations ur5_workstation.launch

Or connect to the hardware workstation with:
	
	roslaunch coro_workstations ur5_workstation.launch robot_ip:=<robot's ip address>
	
## URDF setup

All workstations have been built using URDF files. All of which can be found in coro_workstation/coro_descriptions/urdf/workstations.

**ur5_workstation.urdf.xacro includes:**

- Universal Robot UR5 CB2
- Robotiq 2F-85 gripper
- Robotiq FT300 force-torque sensor
- 3 arms metal table (coro_workstations/coro_descriptions/urdf/common/coro_3arms_table.urdf.xacro)

**ur10_workstation.urdf.xacro includes:** (coming soon...)

- Universal Robot UR10 CB2
- Robotiq 2F-85 gripper
- Robotiq FT300 force-torque sensor
- Large vention table (coro_workstations/coro_descriptions/urdf/common/coro_large_vention_table.urdf.xacro) (comming soon...)
	
**ur5e_workstation.urdf.xacro includes:** (coming soon...)

- Universal Robot UR5 eSeries
- Robotiq 2F-85 gripper
- Robotiq FT300 force-torque sensor (embbeded within the UR5e)
- Small vention table (coro_workstations/coro_descriptions/urdf/common/coro_small_vention_table.urdf.xacro)

## Launch files (coro_workstations/coro_workstations/launch)

* **ur5_workstation.launch:** runs all node required to interact with either the simulated or hardware workstation.

	Simulated workstation
	
	- **`robot_ip`** If this argument isn't provided, the workstation will launch in simulation. Default: `no_ip`.

	Hardware workstation
	
	- **`robot_ip`** By provinding an IP address, the launch file will try to connect to the hardware and Gazebo won't be started. Default: `no_ip`.

* **ur10_workstation.launch:** coming soon...

* **ur5e_workstation.launch:** coming soon...

## Nodes (coro_workstations/coro_workstations/nodes)

You can use the source code of these nodes as a starting point to interact with the workstations. The only differences between the simulated and hardware workstations is the way to interact with the Robotiq 2f-85 gripper (no MoveIt support with the hardware version) and the absence of simulated data from the Robotiq FT300 force-torque sensor.

**Before running any of these nodes with hardware workstations, make sure the robot is free to move a few centimeters from its current position.**

### test_sim_workstation

Makes sure the simulated workstation runs properly. This node:

- makes the robot move a few centemeter from its current position in all axis using MoveIt;
- closes and opens the gripper using MoveIt.

### test_hw_workstation

Makes sure the hardware workstation runs properly. This node:

- makes the robot move a few centemeter from its current position in all axis using MoveIt;
- closes and opens the gripper using ROS topics;
- subcribes to the force-torque sensor ROS topics.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/alexandre-bernier/coro_workstations/issues).

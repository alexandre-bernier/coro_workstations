#!/bin/bash          
#
# Script to copy the right "ros_controller.yaml" file depending if we are using the workstation in simulation or hardware version.
# Arg 1: SOURCE file
# Arg 2: DESTINATION
#

# Copy and replace the yaml file in the moveit_config directory
cp -f $1 $2


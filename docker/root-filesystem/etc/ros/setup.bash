# shellcheck shell=bash disable=SC1090

# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=br0

# # Load the robot's model type and serial number
# source /etc/clearpath-serial.bash

# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
######

# Pass through to the main ROS workspace of the system.
source "/opt/ros/$ROS_DISTRO/setup.bash"

# If you have a catkin workspace, source it below.  Any additional
# environment variables that depend on your workspace should be
# exported here as well.

# TODO: properly fill-out mount and IP/host
export JACKAL_LASER_3D="1"
export JACKAL_LASER_3D_MODEL="vlp16"
export JACKAL_LASER_3D_MOUNT=""
export JACKAL_LASER_3D_HOST=""
export JACKAL_LASER_3D_OFFSET="0 0 0"
export JACKAL_LASER_3D_RPY="0 0 0"

# Front and Rear Fenders
# FIXME: do we have these?
export JACKAL_FRONT_ACCESSORY_FENDER="1"
export JACKAL_REAR_ACCESSORY_FENDER="1"

# Microstrain IMU
export JACKAL_GX5_IMU=1

# # Navsat GPS
# export JACKAL_NAVSAT=1
# export JACKAL_NAVSAT_MODEL=$MODEL
# export JACKAL_NAVSAT_MOUNT=$MOUNT


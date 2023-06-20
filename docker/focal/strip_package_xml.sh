#!/usr/bin/env bash

set -e
set -o pipefail

target="${1}"

# ================================================================
# Garbage
# ================================================================

# Mega-trash ("description" package that pulls in big deps...)
sed -i 's/\(.*cpr_onav_description.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# Trash (sensors we don't use)
sed -i 's/\(.*lms1xx.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*nmea_comms.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*nmea_navsat_driver.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*sick_tim.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*spinnaker_camera_driver.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*pointgrey_camera_driver.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# ================================================================
# Unneeded for base operation
# ================================================================
# Documentation Generation???
sed -i 's/\(.*rosdoc_lite.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# Localization/Mapping
sed -i 's/\(.*gmapping.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*amcl.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*robot_localization.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*map_server.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# ================================================================
# Actually used by RFAL
# ================================================================
# IMU
sed -i 's/\(.*microstrain_inertial_driver.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*imu_filter_madgwick.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
# Hokuyo
sed -i 's/\(.*urg_node.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
# Lidar
# sed -i 's/\(.*velodyne_pointcloud.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
# Motion/Drive
# 
# FIXME: diff_drive_controller needed for jackal_velocity_controller
#        to come up, re-add and test...
sed -i 's/\(.*diff_drive_controller.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*move_base.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
# # Control
# sed -i 's/\(.*joy.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
# sed -i 's/\(.*twist_mux.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# Where is this pulled in? (Answer: IMU calibration script for single call of scipy.optimize)
sed -i 's/\(.*python3-scipy.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# GUI Control
sed -i 's/\(.*interactive_marker_twist_server.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# Clearpath proprietary...
sed -i 's/\(.*jackal_tests.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
sed -i 's/\(.*jackal_firmware.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"


# ================================================================
# Actually needed (by jackal_base)
# ================================================================
# topic direction/throttling utils
# sed -i 's/\(.*topic_tools.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# realtime topic support (for /cmd_drive topic from /jackal_base node)
# sed -i 's/\(.*realtime_tools.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# diagnostic message publisher/aggregator (for /diagnostics and /diagnostics_agg from /jackal_base node)
# sed -i 's/\(.*diagnostic_updater.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
# sed -i 's/\(.*diagnostic_aggregator.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# static/dynamic tf publisher for urdf-defined robot frames
# sed -i 's/\(.*robot_state_publisher.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"
# sed -i 's/\(.*joint_state_controller.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# will get pulled in anyway...
# sed -i 's/\(.*tf.*\)/<!-- NOTE: rfal remove \1 -->/' "${target}"

# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

# stamp MUST use the same ROS time base as laser, IMU and wheel odometry.
# Convert GNSS/GPS time and receiver latency in the driver before publishing.
# frame_id identifies the antenna frame; all satellite coordinates are ECEF.
std_msgs/Header header
# Nominal antenna ECEF position for fixed LOS/Sagnac geometry, not a position factor.
geometry_msgs/Point receiver_position_ecef
hdbot_gnss_msgs/DopplerObservation[] observations

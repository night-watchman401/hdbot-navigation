# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

# Unique constellation/satellite/signal identifier, e.g. GPS/03/L1.
string signal_id
# Positive Doppler means closing range. Measured range rate = -wavelength*doppler.
float64 doppler_hz
float64 wavelength_m
# WGS84 ECEF satellite state consistent with the observation epoch [m, m/s].
geometry_msgs/Point satellite_position_ecef
geometry_msgs/Vector3 satellite_velocity_ecef
float64 satellite_clock_drift_sps
# Standard deviation of range rate in m/s (not Hz).
float64 range_rate_sigma_mps
bool valid

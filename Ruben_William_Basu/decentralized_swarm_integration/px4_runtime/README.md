# PX4 simulation runtime

This directory contains the prebuilt PX4 Software-in-the-Loop runtime used by
EiraX. It allows the UAV launchers to run without the full `PX4-Autopilot`
source repository.

Included Git components:

- `bin/px4`: prebuilt PX4 Software-in-the-Loop flight controller;
- `etc/`: PX4 startup scripts, airframe configuration and metadata;
- `rootfs/`: clean writable PX4 runtime state and flight logs;
- `worlds/default.sdf`: the small world used by the standalone circuit test.

The `models/` directory is required locally but excluded from Git with all
other model and mesh assets. Restore the PX4 x500 and x500-base Gazebo models
from the separately stored simulation/runtime asset package after cloning.

The binary is specific to this Linux installation and its installed Gazebo
Garden / ROS 2 Jazzy libraries. Keep the full PX4 source repository somewhere
else if PX4 must later be rebuilt, upgraded or compiled for real hardware.

# Clearpath Husky UGV

This folder groups UGV-owned software. The Husky retains local authority over wheel motion, obstacle clearance, stopping and Nav2 recovery even when a DJI is the active scout.

## Contents

- `config/`: name, platform, sensors and network inventory.
- `hardware/clearpath/`: Clearpath/Halmstad description and 3D LiDAR attachment.
- `hardware/launchers/commands.py`: known-good Baylands process commands.
- `navigation/config/`: EKF, NavSat, RTAB-Map and generated Nav2 parameters.
- `missions/`: coordinate conversion and robust waypoint execution.
- `monitoring/`: startup topic, message, clock and lifecycle checks.
- `runtime/`: managed child processes and timing support.
- `data_logging/`: run directories and event recording.
- `evaluation/`: localization recording and offline analysis.
- `simulation/`: Baylands-only overrides.

The primary obstacle input is `/husky/lidar3d/points`; the Clearpath velocity input remains `/a201_0000/cmd_vel`. The generated Nav2 configuration defines the Husky footprint and a 1.2 m inflation envelope. It permits up to 2.0 m/s in open terrain, reduces the command by 50% when raw LiDAR obstacles enter a 4 m zone, caps linear speed at 0.5 m/s inside 2 m, and predicts footprint collision two seconds ahead. Normal DWB motion is forward-only; reverse is reserved for short bounded recovery. Mission retries wait for Nav2 lifecycle recovery after a heavy-load stall.

## Physical acceptance

Confirm identity and `robot.yaml`, then test emergency stop, manual low-speed motion, odometry, IMU, GNSS, TF and LiDAR independently. Validate localization, obstacle stopping and Nav2 before enabling peer communication or missions. Gazebo bridges and `simulation/baylands.yaml` must not be used as physical drivers.

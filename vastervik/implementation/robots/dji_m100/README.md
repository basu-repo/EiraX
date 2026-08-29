# DJI Matrice 100 UAV

This folder describes one UAV companion-computer unit. A three-aircraft swarm runs the same software with identities `dji0`, `dji1` and `dji2`.

## Contents

- `config/`: identity, flight envelope, camera and network settings.
- `hardware/`: real DJI SDK/PSDK, telemetry and authority boundary.
- `missions/`: scout, follow, failure, return and landing behavior.
- `navigation/`: UAV-only localization/planning parameters.
- `perception/`: camera calibration, YOLO and observations.
- `simulation/`: M100 model, camera and simulated ENU actuation.
- `ros2_ws/`: historical self-contained reference; ignored by the main build.

Simulation commands absolute ENU `[x,y,z,yaw]` on `/<name>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`. This is DJI-oriented logic, not PX4. YOLO may pitch the gimbal to keep the UGV visible; it does not tilt the entire aircraft or bypass flight safety.

DJI0 begins as scout. Permanent loss transfers the role DJI0 → DJI1 → DJI2, while temporary loss transfers leadership and lets a reconnected aircraft rejoin as follower. If all UAV links fail, the Husky continues with local Nav2.

Physical acceptance order is telemetry, propellers-off camera/gimbal, authority acquire/release, motor-disabled setpoints, controlled low hover, geofence/lost-link/return/landing, one-UAV follow, then multi-UAV separation and failover.

# Husky navigation

This folder owns the Husky's Nav2 stack. Parameters must reflect its skid-steer
kinematics, footprint, ground LiDAR, acceleration and stopping distance.

- `config/`: Nav2, localization and costmap parameters.
- `maps/`: occupancy maps used by this Husky.
- `routes/`: waypoint sequences and mission routes.
- `launch/`: Husky navigation launch files.

Never reuse UAV footprint or altitude parameters here.

# Simulation models

Gazebo-only visual and collision models belong here. Physical URDF and device
configuration remain in each robot's `hardware/` directory.

Currently copied:

- `baylands/`: complete terrain model from the proven standalone simulation.
- `matrice_100/`: Halmstad DJI M100 visual and collision model.
- `zenmuse_z3/`: DJI camera/gimbal visual used by the generated aircraft.
- `husky/`: Gazebo Husky visual/collision and sensor model.
- `goal_marker/`, `waypoint_marker/`, `ground_station/`: mission aids.
- Remaining folders: static Baylands obstacles and scene objects.

Gazebo resolves `model://...` URIs through the resource path configured by the launcher. Each model should keep `model.config`, its SDF, meshes and textures together. Changing only a mesh affects appearance, while changing collision geometry or inertial values affects physics and requires a new movement/safety test.

The packaged physical/ROS descriptions under `src/lrs_halmstad/urdf`, `xacro` and `clearpath` serve a different purpose. Do not replace hardware descriptions merely to repair a Gazebo visual.

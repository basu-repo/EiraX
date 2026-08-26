# File and folder index

This index answers “where is it?” and “what should I edit?” for the active Västervik implementation. Generated datasets and third-party model meshes are described by type rather than listing every binary asset.

## Workspace root

- `README.md`: project purpose, supported runs, architecture and entry points.
- `BUILD.md`: colcon build, overlay and generated-folder explanation.
- `LEGACY_STRUCTURE.md`: history of the earlier robot-oriented layout and migration.
- `run.sh`: original Halmstad command dispatcher for legacy test modes.
- `stop.sh`: stops processes started through that dispatcher.
- `requirements.txt`: Python dependency reference; ROS dependencies remain in `package.xml`/rosdep.
- `colcon.meta`: colcon package build metadata.

## `simulation/`

- `run_baylands_husky.py`: complete Västervik launcher and process owner. It validates assets, creates the dataset, starts Gazebo/ROS, checks health, starts optional DJI coordination, executes Husky legs and shuts down.
- `run_one_dji.py`: generates/validates the embedded M100 and its DJI-compatible simulated actuation and camera.
- `worlds/baylands/baylands_editable.world`: source editable Baylands world.
- `worlds/baylands/BAYLANDS_EDITOR.md`: visual world/spawn editing procedure.
- `worlds/baylands/maps/`: occupancy-map image/YAML pairs.
- `worlds/baylands/routes/`: named route/waypoint YAML files.
- `models/baylands/`: terrain SDF and DAE sections.
- `models/husky/`: Gazebo Husky visual/collision/sensors.
- `models/matrice_100/`: Gazebo M100 visual/collision assets.
- `models/zenmuse_z3/`: camera/gimbal assets.
- Other `models/*`: static Baylands obstacles and markers.

## `robots/husky/`

- `config/identity.yaml`: UGV name, namespace and ROS identity.
- `config/hardware.yaml`: platform/sensor inventory and physical-vs-sim profile values.
- `config/network.yaml`: peer/network interface description.
- `hardware/clearpath/robot.yaml`: copied Clearpath robot and attachment definition.
- `hardware/clearpath/robot.urdf.xacro`: Husky description entry point.
- `hardware/clearpath/lidar3d_0_override.urdf.xacro`: added 3D LiDAR mount/description.
- `hardware/launchers/commands.py`: commands for bridges, EKFs, NavSat, RTAB-Map, Nav2, recorder and mission runner.
- `navigation/config/nav2_config.py`: generates mission-sized Nav2 configuration, footprint, obstacle inflation, speed and bounded recovery.
- `navigation/config/ekf_params.yaml`: global GNSS/odometry EKF.
- `navigation/config/local_ekf_params.yaml`: local continuous odometry EKF.
- `navigation/config/navsat_params.yaml`: GNSS transform settings.
- `navigation/config/ugv_rtabmap_gui.ini`: optional RTAB-Map viewer layout.
- `missions/world_poses.py`: reads named world markers and converts them into the mission's odom coordinates.
- `missions/waypoint_mission.py`: sends goals, monitors completion and waits through Nav2 lifecycle recovery before retrying.
- `monitoring/topic_health.py`: Gazebo, clock, topic, message, finite odometry and lifecycle checks.
- `runtime/process_manager.py`: starts processes with isolated logs and performs controlled shutdown.
- `runtime/monotonic_clock.py`: timing support for simulation processes.
- `data_logging/run_dataset.py`: creates a run folder and appends structured events.
- `evaluation/localization_recorder.py`: records estimator and ground-truth trajectories.
- `evaluation/analyze_localization.py`: offline localization metrics.

## `robots/dji_m100/`

- `config/identity.yaml`: per-aircraft identity template.
- `config/hardware.yaml`: camera, gimbal and DJI hardware contract.
- `config/network.yaml`: UAV/UGV peer network configuration.
- `missions/follow_husky.py`: simulated DJI native ENU scout, escort, formation and landing behavior.
- `missions/failover_protocol.py`: role-state transition helpers.
- `missions/one_dji_check.py`: minimal movement validation.
- `navigation/config/`: UAV-only planning/localization placeholders; never replace Husky Nav2 with these.
- `simulation/baylands.yaml`: simulation-only aircraft/spawn overrides.
- `ros2_ws/`: retained self-contained reference copy, excluded from the main colcon build by `COLCON_IGNORE`.

## `src/lrs_halmstad/`

This is the deployable package. `setup.py` registers executables and package data; `package.xml` declares ROS dependencies.

- `launch/spawn_robot.launch.py`: Halmstad Husky/DJI spawning structure.
- `launch/run_follow.launch.py`: configurable follow/perception composition.
- `launch/support_observation.launch.py`: additional UAV observation nodes.
- `launch/nav2_with_updates.launch.py`, `localization_with_params.launch.py`, `slam_with_params.launch.py`: Halmstad navigation wrappers.
- `lrs_halmstad/coordination/follow_husky.py`: packaged DJI/Husky cooperative controller.
- `lrs_halmstad/coordination/failover_protocol.py`: packaged decentralized failover logic.
- `lrs_halmstad/follow/camera_tracker.py`: produces active gimbal tilt commands using detection/pose information.
- `lrs_halmstad/follow/follow_core.py`: reusable following calculations.
- `lrs_halmstad/follow/follow_uav*.py`: Halmstad UAV following nodes.
- `lrs_halmstad/perception/leader_detector.py`: YOLO detection node.
- `lrs_halmstad/perception/leader_tracker.py`: YOLO tracking variant.
- `lrs_halmstad/perception/leader_estimator.py`: converts visual detections into target estimates.
- `lrs_halmstad/perception/detection_status.py`: standardized detection state/metrics.
- `lrs_halmstad/perception/yolo_common.py`: model loading, image conversion and geometry utilities.
- `lrs_halmstad/perception/onnx_backend.py`: optional ONNX inference backend.
- `lrs_halmstad/sim/simulator.py`: simulation-only UAV behavior adapter.
- `lrs_halmstad/sim/gazebo_model_pose_bridge.py`: Gazebo pose to ROS bridge.
- `lrs_halmstad/nav/ugv_nav2_driver.py`: Halmstad UGV Nav2 driver.
- `lrs_halmstad/vastervik_husky/`: packaged Västervik Husky mission helpers.
- `config/run_follow_defaults.yaml`: detailed Halmstad follow, detector and tracker defaults.
- `config/vastervik_husky/`: packaged Västervik Husky parameters.
- `models/obb/mymodels/baylands-leader-v9-tuned-full.pt`: active YOLO OBB weights.
- `clearpath/`: physical Clearpath-compatible description/configuration.
- `xacro/`, `urdf/`: DJI aircraft, sensors and camera/gimbal description.

## Generated folders

- `build/`: colcon intermediate output. Never edit or deploy as source.
- `install/`: runnable overlay generated from `src/`. Source it; do not edit it.
- `log/`: colcon build/list logs. Useful for build diagnosis, not runtime evidence.
- `datasets/`: experiment output. See `datasets/README.md`.
- `__pycache__/`: disposable Python bytecode cache.

## Supporting folders

- `scripts/`: original Halmstad component launch, audit, recording and plotting utilities.
- `shared/`: documented cross-robot interface boundaries.
- `deployment/`: environment, network and future systemd deployment templates.
- `tools/`: offline analysis, diagnostics and setup helpers.
- `descriptions/`: detailed current state, pipeline, guides, math and handoff notes.

When two files look duplicated, edit the authoritative `src/lrs_halmstad` implementation for physical deployment and update the readable robot-layout equivalent only when the active Baylands launcher imports it. Never treat `install/` as the original.

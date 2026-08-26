# Operational scripts

These utilities are inherited from or compatible with the Halmstad workflow. They provide targeted component tests; the supported Västervik all-in-one entry point is `simulation/run_baylands_husky.py`.

- `run_tmux_1to1.sh`, `stop_tmux_1to1.sh`: original one-to-one stack.
- `run_gazebo_sim.sh`, `run_spawn_uav*.sh`: simulation components.
- `run_nav2*.sh`, `run_localization.sh`: UGV navigation layers.
- `run_teleop.sh`, `publish_twist_stamped.py`: manual command tools.
- `run_rqt_image_view.sh`, `run_nav2_rviz.sh`: visualization.
- `run_record_experiment.sh`, `run_capture_dataset.sh`: recording.
- `capture_omnet_metrics_csv.py`, `plot_network_metrics.py`: network evidence.
- `plot_trajectory_paths.py`: trajectory plots.
- `run_node_audit.sh`, `run_topic_audit_table.sh`: diagnostics.
- `recover_sim_controllers.sh`, `run_kill_all_ros2.sh`: recovery/cleanup.
- `map-making/`: map creation procedures.

Most scripts expect the ROS overlay to be sourced. Inspect their help/header before use. Do not run Gazebo adapters or broad cleanup scripts on a physical robot without confirming their targets.

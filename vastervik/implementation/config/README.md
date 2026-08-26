# Workspace configuration

This folder is for configuration shared by the overall workspace rather than parameters owned by one vehicle.

`rosbag_qos.yaml` provides topic-specific ROS 2 Quality of Service overrides for recording. Camera, sensor and status topics may use different reliability and durability policies; incompatible recorder QoS can discover a topic but receive no data.

Robot-specific values belong elsewhere:

- Husky identity/hardware/network: `robots/husky/config/`.
- Husky Nav2/localization: `robots/husky/navigation/config/`.
- DJI identity/hardware/network: `robots/dji_m100/config/`.
- Deployable package parameters: `src/lrs_halmstad/config/`.

Never commit passwords, DJI credentials, private keys or site secrets. Inject them through the deployment environment.

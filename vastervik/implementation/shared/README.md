# Shared cross-robot contracts

This area defines information both robot types interpret identically while preserving decentralized control.

- `communication/`: discovery, heartbeat and link quality.
- `coordination/`: role vocabulary, handover and task state.
- `interfaces/`: shared ROS message/service/action contracts.
- `monitoring/`: common health and timestamp conventions.
- `safety/`: shared fault names and requested responses.

A shared component may report a disconnected UAV or request a safe return. The UAV still enforces how it returns, and the Husky still enforces how it stops. Vehicle footprints, motor limits, altitude limits, hardware ports and Nav2 parameters do not belong here.

Executable coordination currently lives under `src/lrs_halmstad/lrs_halmstad/coordination/`; this directory documents the deployment boundary.

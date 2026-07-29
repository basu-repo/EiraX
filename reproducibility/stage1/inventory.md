# Stage 1 compatibility and reproduction inventory

Date: 2026-07-26
Status: preservation complete; baseline reproduction not yet started

## 1. Supplied baseline scope

| Baseline | Local source | Preserved evidence | Current readiness |
|---|---|---|---|
| Halmstad ROS 2 system | `halmstad_ws-main/` | 348-file SHA-256 manifest and configuration archive | Source present; original runtime not yet reproduced |
| UAV–UGV network simulation | `UAV_UGV-main/` | 23-file SHA-256 manifest and configuration archive | Source present; required OMNeT++ toolchain absent |
| UAS cybersecurity co-simulation | `1 - UAS_CoSimulation_Seput_Instruction.docx` and `2 - UAS_CoSimulation_Experiment_Instructions.docx` | Document SHA-256 manifest and archived originals | Instructions present; benign 44-column dataset not yet reproduced |

PX4-Autopilot is recorded as a dependency snapshot, not as one of the three
supplied research baselines.

## 2. Provenance state

- `halmstad_ws-main` has no usable local Git repository.
- `UAV_UGV-main` has no usable local Git repository.
- The EiraX root has no usable local Git repository.
- `PX4-Autopilot` is a Git repository at commit
  `78a44ed439ee941acd4844ff8ceaedbfe0faea56`.

Consequently, the manifests establish the current local state only. They cannot
prove which files are unchanged from upstream or identify modifications made
before 2026-07-26. Future work must occur outside the supplied source folders or
be recorded explicitly in `change_log.csv`.

## 3. Host environment

| Component | Recorded value |
|---|---|
| Operating system | Ubuntu 24.04.4 LTS (Noble Numbat) |
| Kernel | Linux 6.8.0-136-generic, x86-64 |
| Robot Operating System | ROS 2 Jazzy Jalisco |
| Gazebo Sim | 8.11.0 |
| Python | 3.12.3 |
| GNU Compiler Collection | 13.3.0 |
| CMake | 3.28.3 |
| Nav2 | 1.3.12 |
| RTAB-Map ROS | 0.22.1 |
| Robot Localization | 3.8.3 |
| ROS–Gazebo bridge | 1.0.22 |

## 4. Communication-simulation dependency audit

| Dependency | Expected by supplied source | Local state |
|---|---|---|
| OMNeT++ | Required | Missing |
| `opp_run` / `opp_makemake` | Required | Missing |
| INET 4.5 | `../inet4.5` | Missing |
| FLORA | `../flora` | Missing |
| Simu5G 1.4.2 | `../Simu5G-1.4.2` | Missing |

The `UAV_UGV-main` Makefile explicitly links against INET 4.5, FLORA and
Simu5G 1.4.2. Stage 3 cannot be claimed until compatible versions are installed,
the project builds, and each communication configuration produces repeatable
metrics.

## 5. Preserved configuration scope

The configuration archive contains:

- `halmstad_ws-main/config`;
- `halmstad_ws-main/maps`;
- `halmstad_ws-main/src/lrs_halmstad/config`;
- `halmstad_ws-main/requirements.txt`;
- `UAV_UGV-main/omnetpp.ini`;
- `UAV_UGV-main/ground.xml`;
- `UAV_UGV-main/Makefile`;
- the principal UAV–UGV NED network descriptions; and
- both UAS co-simulation instruction documents.

The complete source trees remain protected by per-file manifests.

## 6. Existing evidence that must not be mistaken for reproduction

The supplied folders contain launch scripts, bridge implementations, eight-field
OMNeT metrics code, maps and analysis utilities. Their presence shows intended
functionality, not successful reproduction on this host. No Stage 2–4 claim will
be made until runtime commands, versions, topics, ports, logs and results are
captured under controlled baseline conditions.

The independent EiraX prototype has demonstrated a successful Husky mission using
Nav2, three-dimensional LiDAR, RTAB-Map and localization. It has also demonstrated
PX4 UAV flight and aerial mapping. The latest cooperative UAV test is not a
successful baseline: unstable aerial LiDAR odometry caused transform drops,
invalid flight setpoints, a tree collision and PX4 failsafe.

## 7. Separation policy

- Supplied baseline folders are read-only reference inputs from this point.
- New adapters, repaired interfaces and experiments belong under `integration/`.
- A necessary edit that cannot be implemented externally must first be copied
  into the integration area and entered in the change log.
- Generated datasets and build products are not source baselines.
- Ground truth may be recorded for evaluation but must not control either robot.

## 8. Stage 1 exit decision

Stage 1 preservation requirements are satisfied for the state currently available:

- source inputs identified;
- host and software state recorded;
- missing dependencies recorded;
- per-file checksums generated;
- original configuration snapshot archived;
- separate integration area created; and
- classified change log started.

The next roadmap task is Stage 2: reproduce `halmstad_ws-main` under its original
Clearpath UGV, `dji0` UAV, following controller and original world assumptions.

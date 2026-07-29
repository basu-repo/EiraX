# EiraX integration area

This directory is the only location for source-first compatibility adapters and
cross-baseline integration work.

The supplied references remain in:

- `../halmstad_ws-main`
- `../UAV_UGV-main`
- `../1 - UAS_CoSimulation_Seput_Instruction.docx`
- `../2 - UAS_CoSimulation_Experiment_Instructions.docx`

Rules:

1. Do not edit a supplied baseline merely to make it fit EiraX.
2. Reproduce original behaviour before adapting interfaces.
3. Copy or wrap only the minimum component required for integration.
4. Classify every change in
   `../reproducibility/stage1/change_log.csv`.
5. Store commands, versions, logs and expected outcomes with each experiment.
6. Keep measured features separate from scenario labels.
7. Never use simulation ground truth as a navigation input.

Planned subdirectories will be created only when their corresponding baseline has
been reproduced:

```text
integration/
  halmstad_adapter/
  omnet_bridge/
  uas_cosimulation/
  eirax_interfaces/
```

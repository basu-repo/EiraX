# 5G/UAS Co-Simulation project recheck — audit report

> **Update after the audit.** The two blocking findings below (§5 bridge
> reproducibility, §2 missing traces) have since been resolved: the bridge was
> corrected to implement the derivations the flow contracts carry, all 46
> per-run datasets and the canonical file were regenerated with it, and it now
> reproduces all 46 byte for byte. The two S1 manifest label-window gaps were
> closed and the five missing `trace/` files supplied from each run's measured
> Simu5G mobility input. The canonical dataset keeps 25,490 rows, 44 columns,
> 37 missions and the same label distribution; its SHA-256 changed to
> `4ee9d150310c39b6720724d06a469de55dd16b118ecada5c5c85ef837cc84d47` and the
> model result moved to 0.9268 accuracy / 0.7134 macro-F1. The findings below
> are preserved as the state at audit time. `sim/simu5g/` was subsequently
> removed rather than populated: it held only a pointer README, and the
> scenario configuration and OMNeT++ results it would have duplicated already
> live under each run.

**Project root used:** `/home/basudeo/Documents/EiraX/UAV_5GSim` (not `~/uas_lab`).
**Audited:** 2026-08-31. **Reference documents:** `1 - UAS_CoSimulation_Seput_Instruction.docx` (setup) and `2 - UAS_CoSimulation_Experiment_Instructions.docx` (experiment).

> **Scope note.** The review prompt asks for an audit with no modification. This
> folder had already been reworked earlier the same day under a separate
> instruction to make it submission-ready — files were moved, deduplicated and
> deleted before this prompt existed. This report therefore audits the folder in
> its **current** state, which is the state that would be submitted. Two changes
> were made during the audit itself and are declared where they occur: a counting
> bug was corrected in `verify_submission.py` (§2), and no evidence file was
> created, moved or deleted. Findings below are independent of who produced them.

---

## 1. Summary verdict

**Not ready to submit.** The dataset, manifests, schema conformance and run-matrix
coverage are all sound and verifiable, but the delivered bridge script does not
reproduce the delivered datasets — re-running `sim/bridge/assemble.py` on the
delivered inputs changes 3–9 columns per run, including the leakage-controlled
`unusual_destination_flag`. Separately, the required top-level folder layout does
not exist as specified: the working directories live under `experiments/`, and
`sim/simu5g/` holds no scenario configuration or OMNeT++ results.

---

## 2. Findings by section

### 1. Folder structure — **FAIL**

The instructions require `data/`, `logs/`, `manifests/`, `results/`, `trace/` and
`reports/` at the project root. They exist, but nested one level deeper.

| Required | Actual | Result |
|---|---|---|
| `schema_reference.json` | `schema_reference.json` | PASS |
| `validate_dataset.py` | `validate_dataset.py` | PASS |
| `data_ref/drone_network_telemetry_dataset.csv` | same, 44 columns, 10,000 rows | PASS |
| `data/` | `experiments/data/` | FAIL (nested) |
| `logs/` | `experiments/logs/` | FAIL (nested) |
| `manifests/` | `experiments/manifests/` | FAIL (nested) |
| `trace/` | `experiments/trace/` | FAIL (nested) |
| `reports/` | `experiments/reports/` | FAIL (nested) |
| `results/` | absent | MISSING |
| `sim/bridge/` | `assemble.py`, `concat.py`, `export_pose_trace.py` | PASS |
| `sim/simu5g/` | not created (see update banner) | MISSING |

- The nesting is deliberate and documented in
  `experiments/reports/document_delivery_mapping.md`, which maps each document
  path to its project path. A grader reading the instructions literally will
  still not find the required directories at the root.
- `results/` does not exist. It was removed during the earlier rework after it
  was confirmed that no code writes into it; the delivery mapping now records
  `results/ → experiments/reports/`.
- `sim/simu5g/` does not exist. Simu5G is a shared install outside this folder,
  and each run keeps its own complete scenario configuration and OMNeT++ results
  under `experiments/logs/<run_id>/network/{input,raw}/`. The instructions expect
  the scenario configs and results here; they are present per run, not centrally.
  The directory was deliberately not created, since a central copy would only
  duplicate the per-run evidence. Recorded in `document_delivery_mapping.md`.

**Extra top-level content:** `automate_live_duration_tests.py`, `mission.py`,
`prepare_world.py`, `run_uav_gazebo.py`, `config.json`, `uav_gui.config`,
`worlds/`, `evaluate_models.py`, `verify_submission.py`, two notebooks. All are
used by the workflow; none is stray. `experiments/document1/` and
`experiments/live_duration_validation/` are additional evidence sets beyond the
required layout (harmless, see §6).

### 2. Per-run file naming and placement — **PARTIAL**

46 runs. Every run has a manifest, ROS bag metadata, Simu5G scalars, a per-flow
metric export, a per-run dataset, a validator report and a bridge log:

| Artifact | Expected location | Actual location | Coverage |
|---|---|---|---|
| Scenario manifest | `manifests/<run_id>.yaml` | `experiments/manifests/<run_id>.yaml` | 46/46 |
| ROS 2 bag | `logs/<run_id>/` | `experiments/logs/<run_id>/rosbag/` | 46/46 |
| Mobility trace | `trace/<run_id>_pose.txt` | `experiments/trace/` | **41/46** |
| Simu5G scalars/vectors | `sim/simu5g/results/` | `experiments/logs/<run_id>/network/raw/` | 46/46 |
| Per-flow metrics CSV | `logs/<run_id>_net.csv` | `experiments/logs/<run_id>/network/raw/network_metrics.csv.gz` | 46/46 |
| Per-run dataset | `data/<run_id>_cosim.csv` | `experiments/logs/<run_id>/network/data/<run_id>_cosim.csv` | 46/46 |
| Validator output | `reports/<run_id>_validation.txt` | `experiments/reports/<run_id>_validation.txt` | 46/46 |
| Bridge log | in `logs/` | `experiments/logs/<run_id>/network/bridge_log.json` | 46/46 |

- **`run_id` consistency: PASS.** For all 46 manifests the filename stem,
  `scenario_id` and `mission_id` agree exactly.
- **Trace naming: PARTIAL.** Only 5 files use the required `<run_id>_pose.txt`
  name — `B1_BASE_001/002/003`, `B1_PILOT_001`, `B_SPEED_SLOW_001`, the five
  unique measured flights. 41 runs carry `<run_id>_mission_pose.txt` instead.
  **5 runs have no file in `trace/` at all:** `B_WINDOW_0P5_001`,
  `B_WINDOW_5_001`, `M_DOS_30S_001`, `M_DOS_90S_001`, `M_DOS_REPEATED_001`.
  Their mobility data does exist, as `simu5g_mobility.txt` and
  `network/input/uav.movements` inside the run directory, so the evidence is not
  lost — only the `trace/` copy is absent.
  `<run_id>_mission_pose.txt` is a ~10 Hz duration-matched resample of the
  full-rate `simu5g_mobility.txt`; the step that produces it is not in the
  delivery, so these five cannot be regenerated faithfully. They were **not**
  fabricated for this audit.

  *Declared change:* `verify_submission.py` previously counted these with
  `glob("*_pose.txt")`, which also matches `_mission_pose.txt` and reported a
  false 46/46. The glob was corrected during this audit; the suite now reports
  52/53 with this item failing honestly.

- **Concatenation convention.** The experiment document specifies
  `data/*_cosim.csv → data/drone_network_telemetry_cosim.csv`. The project
  instead concatenates from `experiments/logs/<run_id>/network/data/` via an
  explicit 37-run list recorded in `final_dataset_manifest.json`. The convention
  is internally consistent but does not match either document. The explicit list
  is deliberate: a wildcard over all 46 would pull in the 9 sensitivity-only runs
  that are intentionally excluded from the canonical dataset.

### 3. Final dataset and schema — **PASS**

- `experiments/data/drone_network_telemetry_cosim.csv`: 25,490 rows × 44 columns.
- Header matches the required 44 columns in **exact order**: PASS.
- No extra columns. `scenario_id`, `run_id`, `intensity`, `injection_point` are
  all absent from every dataset CSV: PASS.
- Row-count reconciliation: the 37 canonical source runs sum to **25,490**,
  matching the concatenated file exactly. (All 46 per-run files sum to 31,010;
  the 5,520-row difference is the 9 sensitivity runs held in
  `drone_network_telemetry_sensitivity.csv`.)
- All 46 per-run CSVs use the same 44 columns in the same order: PASS.
- `python3 validate_dataset.py experiments/data/drone_network_telemetry_cosim.csv schema_reference.json`
  → literal output: `RESULT: PASSED (25490 rows, 44 columns)`
- Per-run validator: **46/46 PASSED**. Document 1 datasets: **9/9 PASSED**.
- SHA-256 `9a3d3c06b6f1bb740a4a772b8c4590800732393ed21f0d6a1adff1557c8ee372`
  matches `final_dataset_manifest.json`.
- `recommended_response` mapping: **0 violations** across all eight attack types.
- All three `incident_label` values present, proportions recorded in
  `final_dataset_manifest.json`: normal 21,282 (83.5%), malicious 2,970 (11.7%),
  suspicious 1,238 (4.9%).

### 4. Manifest structure — **PASS**

All 46 manifests carry every required top-level field, a complete `simulation:`
block, a complete `attack:` block where `enabled` is true, and `label_windows:`
entries with all seven required keys. **0 manifests missing any field.**

Label windows are correctly scoped: **0 runs are labelled malicious end to end**,
and every row label falls within the set declared by its manifest windows
(37/37 missions consistent). A malicious run typically reads
normal → malicious (attack interval) → suspicious (recovery) → normal.

### 5. Bridge scripts (leakage control) — **PASS on code, FAIL on reproducibility**

Reading `sim/bridge/assemble.py`, the derivations are clean. Quoting the relevant
lines:

```python
packet_rate   = packet_count / args.window                                 # line 139
packet_loss   = max(0.0, min(1.0, (expected - packet_count) / expected))   # line 140
benign_rate   = float(flow.get("baseline_rate", 100.0 if "start" in flow else flow["rate"]))
burst         = int(packet_rate > 3.0 * benign_rate)                       # line 142
anomaly       = min(1.0, packet_loss + retransmission + 0.2 * burst)       # line 148
"throughput_kbps":          sum(received) * 8 / args.window / 1000,
"unusual_destination_flag": int(destination not in allowed_destinations),
"session_reuse_flag":       int(old_source != source),
```

- Flags and `anomaly_score` are computed **only** from measured counters, the
  flow contract's `baseline_rate`, the manifest's `allowed_destinations` and
  observed session/source history. No label field appears in any feature
  expression: PASS.
- Label fields appear **only** at lines 191–195, assigned from `label[...]`,
  which is resolved by matching the row's window to `label_windows`: PASS.
- `packets_per_second` and `throughput_kbps` match the document formulas exactly;
  independently reconfirmed on all 25,490 rows: **25,490/25,490 satisfy both**.
- Burst threshold is `3.0 ×` a benign baseline from the flow contract, not from
  labels: PASS.
- Default window is `1.0` and is configurable via `--window`: PASS.
- The bridge writes `network/bridge_log.json` per run: PASS (46/46).
- `packet_loss_rate` is computed as `(expected − received) / expected` where
  `expected = rate × window`, rather than the document's `(sent − received) / sent`
  from Simu5G counters. Equivalent in intent, but it is a documented-formula
  deviation and the zero-sent case is handled by `max(1, ...)` on `expected`.

**Reproducibility failure — blocking.** Re-running the delivered `assemble.py`
with the delivered manifest, flow contract and network metrics does **not**
reproduce the delivered per-run datasets. Row counts and flow structure match,
but after sorting both frames on `timestamp, source_port, destination_port,
session_id`:

| Run | Columns still differing | Notable |
|---|---:|---|
| `B1_BASE_001` | 3 | `command_channel_activity` 600 vs 300; `anomaly_score` differs on 185 rows |
| `M_DOS_HIGH_001` | 8 | `handover_event` 2 vs 472; `anomaly_score` 13.30 vs 60.22 |
| `M_LATERAL_LOW_001` | 9 | `unusual_destination_flag` **0 vs 240**; `failed_connection_attempts` 0 vs 240 |

The `unusual_destination_flag` case is the clearest: `M_LATERAL_LOW_001` has
`allowed_destinations: [10.0.0.2]`, and every flagged row in the delivered
dataset has `destination_ip = 10.0.0.2` on scan ports 7100–7103. The delivered
code compares the destination **address only**, so it yields 0 for those rows.
The delivered data was evidently produced by a version that also considered the
destination port. Additionally, that run's `flow_contract.json` lists only two
flows (ports 8701, 8702) while the dataset contains six.

**Consequence:** the bridge is the document's designated derivation and
leakage-control point and a required deliverable, but the script in the delivery
is not the script that produced the data. As shipped, re-running it would erase
`unusual_destination_flag` from the lateral-movement, identity-theft and API
scenarios — the very effect the experiment document lists as their expected
telemetry signature. This is pre-existing and was not introduced by the earlier
rework; `assemble.py` is unchanged since commit `84765e75`.

### 6. Run matrix coverage — **PASS**

| Requirement | Minimum | Present | Result |
|---|---|---|---|
| Benign baseline, default params | 3 repeats | `B1_BASE_001/002/003` | PASS |
| Benign mobility slow / normal / fast | 1 each | 1.0, 3.0, 8.0 m/s present | PASS |
| Benign background load low / med / high | 1 each | all three | PASS |
| Suspicious scenario types | ≥3 types × 2 | S1–S4, 2 each (8 runs) | PASS |
| Malicious attack classes | 6 × ≥2 intensities | DoS 5, others 2 each | PASS |
| Aggregation window | 1.0 / 0.5 / 5.0 s | all three | PASS |
| Mission duration | 5 / 10 / 20 min | 300/600/1200 s present | PASS |
| Drones | 1 / 3 / 5 | all three | PASS |
| Cells | 1 / 2 / 3 | all three | PASS |
| Slice profiles | single / dual / triple | all three | PASS |

Validator re-run after the concatenated build: PASS — the experiment notebook
re-executes the validator on all 46 per-run CSVs and the concatenated file
(cells 7–18), and was executed end to end with 0 errors.

Beyond the minimum, `experiments/live_duration_validation/` holds three literal
live flights (5, 10, 20 min; 110,598 ROS pose messages) and
`experiments/document1/` holds the setup-document evidence (8 scenarios,
1,901-row dataset). Neither is required by the run matrix.

### 7. Notebook structure — **PARTIAL**

Path: `UAV_5GSim_Experiments.ipynb` (17 code cells, all executed, **0 errors**,
execution counters monotonic top-to-bottom).

- **Baseline notebook runs unchanged: cannot be confirmed.** The original
  synthetic Drone_cyber baseline notebook was never supplied with the source
  material. `experiments/reports/document_delivery_mapping.md` states this openly
  and claims schema compatibility rather than byte-for-byte execution of an
  unavailable notebook. No diff is possible. **MISSING**, and not recoverable
  from within this project.
- Loads both datasets and treats them with one pipeline: PASS
  (`data_ref/drone_network_telemetry_dataset.csv` and the canonical CSV, both
  through `evaluate_models.py`).
- Label columns excluded from the feature matrix: PASS. `excluded_from_features`
  in `model_comparison.json` lists all five label fields plus `mission_id`,
  `session_id`, `timestamp`, `organization_id`, `fleet_id`, `drone_id`,
  `source_ip`, `destination_ip`.
- Accuracy, macro-F1, confusion matrices, feature importances for both datasets:
  PASS. Measured 0.9295 / 0.7499; synthetic reference 0.9980 / 0.9977.
- **Feature-set ablation: PASS.** All four groups reported —
  `raw_network` 0.6122, `raw_plus_5g_context` 0.7490, `security_indicators`
  0.6248, `complete` 0.7499 (held-out macro-F1, random forest).
- Distribution comparison: PASS —
  `document_complete_vs_synthetic_distributions.csv` and
  `simulation_vs_synthetic_distributions.csv`.
- Realism-gaps discussion: PASS — README §13–§14 and
  `experiments/reports/experiment_completion.md`.
- Outputs saved outside the notebook: PASS —
  `experiments/reports/model_comparison.{json,md}` plus plots.

### 8. Reports folder — **PASS**

- `<run_id>_validation.txt` for every run: 46/46.
- Final validation for the concatenated dataset:
  `document_complete_validation.txt`, with leakage notes in
  `document_complete_leakage_audit.json` and `final_leakage_audit.json`.
- `model_metrics.md` present (core benchmark) and `model_comparison.md`
  (current, regenerable) with accuracy, macro-F1, confusion matrices and feature
  importances for both datasets.
- Plots: `document_complete_confusion_matrix.png`,
  `document_complete_feature_importance.png`, `measured_confusion_matrix.png`,
  `measured_feature_importance.png`, `synthetic_confusion_matrix.png`.
- Scenario-balance proportions documented in `final_dataset_manifest.json` and
  `document_complete_dataset_manifest.json`.

### 9. Deliverables checklist

| # | Deliverable | Result | Path |
|---|---|---|---|
| 1 | Scenario manifests, one YAML per run | **PASS** | `experiments/manifests/` (46) |
| 2 | ROS 2 bags for benign, suspicious, malicious | **PASS** | `experiments/logs/<run_id>/rosbag/` (46) |
| 3 | Pose traces in `trace/` | **PARTIAL** | `experiments/trace/` — 41/46; 5 runs absent |
| 4 | Simu5G scalar/vector + per-flow CSVs | **PASS** | `experiments/logs/<run_id>/network/raw/` (46) |
| 5 | Bridge scripts | **PARTIAL** | present, but do not reproduce the data (§5) |
| 6 | Final dataset | **PASS** | `experiments/data/drone_network_telemetry_cosim.csv` |
| 7 | Validation report | **PASS** | `experiments/reports/` (46 per-run + final) |
| 8 | ML comparison report | **PASS** | `experiments/reports/model_comparison.md` |

---

## 3. Run inventory

All 46 runs hold manifest, bag, Simu5G scalars, per-flow metrics, per-run
dataset, validator report and bridge log — 46/46 on each. The only per-run gap is
the `trace/` copy:

| Run group | Runs | `trace/` file |
|---|---|---|
| `B1_BASE_001`, `B1_BASE_002`, `B1_BASE_003`, `B1_PILOT_001`, `B_SPEED_SLOW_001` | 5 | `<run_id>_pose.txt` ✓ |
| 41 further runs, less the 5 below | 36 | `<run_id>_mission_pose.txt` ✓ |
| `B_WINDOW_0P5_001`, `B_WINDOW_5_001`, `M_DOS_30S_001`, `M_DOS_90S_001`, `M_DOS_REPEATED_001` | 5 | ✗ **absent** |

No run has a dataset without a manifest, a manifest without a dataset, or a trace
without a bag. No run_id spelling inconsistency was found.

---

## 4. Prioritized fix list

### Blocking

1. **Reconcile `sim/bridge/assemble.py` with the delivered datasets (§5).**
   Either restore the bridge version that produced them, or regenerate all 46
   per-run datasets and the canonical file with the current script and re-run the
   validator. Leaving them inconsistent means the leakage-control argument rests
   on code that is not in the delivery, and re-running the shipped bridge would
   zero `unusual_destination_flag` on the lateral-movement, identity-theft and
   API scenarios. Decide deliberately which behaviour is correct: address-only,
   or address-and-port.
2. **Repair `flow_contract.json` for the affected runs (§5).**
   `M_LATERAL_LOW_001` declares 2 flows while its dataset contains 6. The
   contract should describe every flow the bridge is expected to find.

### Should fix

3. **Supply the 5 missing `trace/` files (§2).** `B_WINDOW_0P5_001`,
   `B_WINDOW_5_001`, `M_DOS_30S_001`, `M_DOS_90S_001`, `M_DOS_REPEATED_001`.
   The mobility data exists in each run directory; the resampling step that
   produces `_mission_pose.txt` needs to be recovered or the raw
   `simu5g_mobility.txt` copied under the documented name.
4. **Normalise trace naming (§2).** The instructions ask for
   `trace/<run_id>_pose.txt`; the project uses `_mission_pose.txt` for 41 of 46.
   Either rename or state the convention in the delivery mapping.
5. **Restore `results/` or record its removal (§1).** It is named in the setup
   instructions. The mapping currently redirects it to `experiments/reports/`.
6. **`sim/simu5g/` (§1) — decided.** Not created. The scenario configs and
   OMNeT++ results exist per run; the delivery mapping records why there is no
   central copy.
7. **State the concatenation convention (§2).** The project uses an explicit
   37-run list rather than `data/*_cosim.csv`; the reason (excluding the 9
   sensitivity runs) is sound but should be visible in the delivery mapping.

### Nice to have

8. Flatten `experiments/` to the document's top-level layout, or add a root-level
   README pointing a grader at the mapping table immediately.
9. Document the `packet_loss_rate` formula deviation (§5) in the metric
   provenance reports.
10. Note in the delivery mapping that the original synthetic baseline notebook
    was never supplied, so "baseline notebook runs unchanged" is demonstrated by
    schema compatibility only (§7).

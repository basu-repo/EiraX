# Stage 1 preservation package

Created: 2026-07-26
Project root: `/home/basudeo/Documents/EiraX`

This package freezes the observable state of the three supplied baselines before
new source-first integration work:

1. `halmstad_ws-main`
2. `UAV_UGV-main`
3. The two supplied UAS co-simulation instruction documents

The source folders do not contain usable Git metadata. Therefore, this package
must not be described as proof of their original upstream state. It records the
exact state available locally on 2026-07-26, after earlier exploratory work may
already have occurred.

## Contents

- `inventory.md`: baseline scope, environment, dependencies, limitations and
  readiness decision.
- `change_log.csv`: classification of preserved material, compatibility repairs,
  EiraX extensions and candidate novelty.
- `manifests/*.sha256`: per-file SHA-256 manifests and artifact checksums.
- `snapshots/supplied_configurations_20260726.tar.gz`: configuration snapshot.

## Verification

Run from the EiraX project root:

```bash
sha256sum -c reproducibility/stage1/manifests/halmstad_ws-main.sha256
sha256sum -c reproducibility/stage1/manifests/UAV_UGV-main.sha256
sha256sum -c reproducibility/stage1/manifests/UAS_CoSimulation_documents.sha256
sha256sum -c reproducibility/stage1/manifests/stage1_artifacts.sha256
```

The configuration archive preserves paths relative to the project root. Inspect
without extracting:

```bash
tar -tzf reproducibility/stage1/snapshots/supplied_configurations_20260726.tar.gz
```

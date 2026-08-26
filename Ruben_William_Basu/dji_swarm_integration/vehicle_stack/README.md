# Copied cooperative vehicle stack

This is a physical, integration-local copy of the accepted one-Husky/one-PX4
cooperative runtime. Its authoritative entry point is:

```bash
python3 ../scripts/run_baylands_swarm.py
```

It resolves the world, models, PX4 runtime, ground stack, datasets and configs
from the parent isolated project. It does not import the original workspace.

Do not add a second PX4 controller while this stack's UAV follower is active.
See [../RUNBOOK.md](../RUNBOOK.md) for operating instructions.

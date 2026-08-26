# Robot-specific software

This layout separates software by the computer that owns the final safety decision. `husky/` represents the UGV computer; `dji_m100/` represents a UAV companion computer. A deployment can copy the relevant robot folder and required shared contracts without copying the other vehicle's controllers.

Each robot follows the same layers: `config/` for identity and inventory, `hardware/` for drivers and adapters, `navigation/` for local motion, `missions/` for high-level tasks, and `simulation/` for explicit hardware replacements.

Keep vehicle limits local. Husky footprint and braking distance never belong in DJI parameters. DJI altitude and landing rules never belong in Husky Nav2. `shared/` defines message meaning; it must not become a centralized motor controller.

The authoritative colcon package is `src/lrs_halmstad`. Parts of this directory are readable references and known-good simulation helpers, so check the root ownership guide before modifying duplicate-looking files.

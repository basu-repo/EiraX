# Persistent Baylands world

This setup keeps one editable world file:

`simulation/worlds/baylands/baylands_editable.world`

It initially contains the Baylands model and Husky at pose
`46 -23 1 0 0 0` (`x y z roll pitch yaw`). Model directories are not changed by
the editor or save scripts.

## Open

From the Västervik implementation folder:

```bash
./simulation/worlds/baylands/open_baylands_editor.sh
```

Gazebo starts paused so the Husky does not roll away while arranging the world.
Use the translate and rotate controls to move it, and the Insert Resources panel
to add models.

## Save

Keep Gazebo open. In a second terminal, from the Västervik implementation
folder, run:

```bash
./simulation/worlds/baylands/save_baylands_world.sh
```

This asks the running Gazebo server to overwrite `baylands_editable.world` with
the current world state. The next call to `open_baylands_editor.sh` therefore
opens the last explicitly saved state.

Saving is explicit: merely closing Gazebo does not reliably save world edits.
Run the save command after each set of changes and before closing Gazebo.

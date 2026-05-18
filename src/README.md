# MacGyvBot Multi-Package Migration

This directory is the staging area for moving the current single `macgyvbot`
ROS 2 package toward a workspace-style multi-package layout.

The packages under `src/` are now the active migrated runtime path. The
repository root is a colcon workspace root, not a ROS package.

Do not reintroduce production logic or ROS package metadata at the repository
root. Each migration step should move one responsibility, update its
launch/build/test contract, and remove the old import path after the new
package is verified.

## Target Package Layout

```text
src/
  macgyvbot_interfaces/
  macgyvbot_config/
  macgyvbot_domain/
  macgyvbot_resources/
  macgyvbot_bringup/
  macgyvbot_command/
  macgyvbot_perception/
  macgyvbot_manipulation/
  macgyvbot_task/
  macgyvbot_ui/
```

## Migration Order

1. Define shared ROS interfaces.
2. Move launch/config wiring into bringup.
3. Move command input.
4. Move perception nodes and helpers.
5. Move manipulation/control adapters.
6. Move task orchestration.
7. Move UI or keep it with command until it grows.

Keep new production code in the package that owns the relevant responsibility.

Current status:

- `macgyvbot_interfaces` has initial typed message contracts for command,
  feedback, task status, hand grasp result, and tool mask lock payloads.
- `macgyvbot_config` owns shared Python runtime constants.
- `macgyvbot_domain` owns shared in-process Python dataclasses.
- `macgyvbot_resources` owns shared calibration and model asset installation.
- `macgyvbot_bringup` owns the primary `macgyvbot.launch.py` entrypoint.
  `macgyvbot_compat.launch.py` is a package-local alias for transition testing.
- `macgyvbot_command` now owns the command input node, parser, STT/TTS helpers,
  and command-coupled GUI widget.
- `macgyvbot_perception` now owns perception helpers and the hand grasp
  detection node for the migrated launch path.
- `macgyvbot_manipulation` now owns robot control helpers.
- `macgyvbot_task` now owns the main task node and application workflows for
  the migrated launch path.
- UI package is still a future boundary; command-coupled UI currently lives in
  `macgyvbot_command`.

## Package Responsibilities

- `macgyvbot_interfaces`: shared `.msg`, `.srv`, and `.action` definitions.
- `macgyvbot_config`: shared Python runtime constants.
- `macgyvbot_domain`: shared in-process Python dataclasses.
- `macgyvbot_resources`: shared calibration and model asset installation.
- `macgyvbot_bringup`: launch files, runtime composition, and package-level configuration wiring.
- `macgyvbot_command`: command input, parser, validation, STT/TTS, and command publication.
- `macgyvbot_perception`: camera input, detection, depth projection, pick target resolution, and hand-grasp perception.
- `macgyvbot_manipulation`: MoveIt, gripper, force sensing, safe workspace, and low-level robot adapters.
- `macgyvbot_task`: pick/return workflows, command routing, status publishing, and task-domain models.
- `macgyvbot_ui`: operator UI widgets and UI-facing presenters.

# macgyvbot_config

Shared Python runtime constants for the migrated MacGyvBot packages.

## Scope

- frame, link, and joint pose constants
- ROS topic names
- model and weight filenames
- pick, handoff, return, grasp, timing, UI, and VLM constants

This package owns Python constants imported by multiple runtime packages. Launch
files and YAML runtime composition still belong in `macgyvbot_bringup`.

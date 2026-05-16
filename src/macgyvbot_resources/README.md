# macgyvbot_resources

Shared non-code assets for migrated MacGyvBot packages.

## Scope

- `calibration/`
- `weights/`
- `weights/vlm/`

This package owns source-tree calibration and model asset locations under the
ROS workspace `src/` tree. Runtime packages should query
`macgyvbot_resources`; the repository root no longer owns installed model or
calibration assets.

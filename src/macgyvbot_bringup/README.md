# macgyvbot_bringup

Launch and runtime configuration package for MacGyvBot.

## Planned Scope

- System launch files
- Per-package launch composition
- YAML runtime parameters
- RViz config
- System readiness checks

## Migrated From

```text
launch/
config/
```

## Current State

`launch/macgyvbot.launch.py` is the primary migrated launch file. It starts
executables from `macgyvbot_task`, `macgyvbot_perception`, and
`macgyvbot_command`.

`launch/macgyvbot_compat.launch.py` is a package-local alias that delegates to
the primary migrated launch.

Migrated launch defaults resolve model files from `macgyvbot_resources` and
MoveItPy YAML from `macgyvbot_bringup`.

Model launch arguments default to files installed in `macgyvbot_resources`, for
example `share/macgyvbot_resources/weights/yolov11_best.pt`. Override
`yolo_model`, `grasp_model`, or `sam_checkpoint` at launch time when using
external model artifacts.

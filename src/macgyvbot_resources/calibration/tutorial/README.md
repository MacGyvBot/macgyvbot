# Calibration Tutorial Scripts

This folder keeps the minimal scripts needed to collect, regenerate, and
validate the hand-eye calibration artifact in `../T_gripper2camera.npy`.

The scripts were imported from the external calibration tutorial material. They
are hardware-specific utilities, not normal MacGyvBot runtime nodes.

## Files

- `data_recording.py`: captures chessboard images and matching Doosan robot
  poses into `data/calibrate_data.json`.
- `handeye_calibration.py`: computes `T_gripper2camera.npy` from the captured
  chessboard images and robot poses.
- `eye2hand_calibration.py`: alternate eye-to-hand calibration reference from
  the original tutorial.
- `test.py`: manual click-to-pick validation script for checking camera-to-base
  projection with a generated artifact.
- `realsense.py`: RealSense image helper used by `test.py`.
- `onrobot.py`: OnRobot RG gripper helper used by `test.py`.

## Notes

These scripts assume the original lab setup and contain hardware-specific
values such as robot ID, robot model, camera device number, gripper name,
Z offsets, and tool-changer IP. Review those values before running anything
against hardware.

The generated `T_gripper2camera.npy` should be copied one level up to
`src/macgyvbot_resources/calibration/T_gripper2camera.npy` only after validating
the transform with a known camera point.

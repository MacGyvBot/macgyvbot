# macgyvbot Calibration

This folder stores calibration artifacts and tutorial scripts used by
`macgyvbot`.

- `T_gripper2camera.npy`: current 4x4 hand-eye calibration artifact.
- `tutorial/`: minimal capture, solving, and validation scripts for regenerating
  this artifact.

## Current Artifact

`T_gripper2camera.npy` stores the fixed camera pose relative to the Doosan
gripper/TCP frame for the current physical camera mount.

The runtime composes this fixed artifact with the current end-effector pose to
convert camera-frame detections into robot base-frame targets:

```python
T_base2camera = T_base2gripper @ T_gripper2camera
point_base = T_base2camera @ point_camera
```

In this repository, the `.npy` file is the source of truth. Do not copy the
numeric matrix into this README, because duplicated matrix values can drift from
the actual runtime artifact.

## Calibration Workflow

The checked-in artifact was generated from the depth-camera calibration tutorial
workflow preserved under `tutorial/`. The workflow uses:

- chessboard image captures;
- matching robot poses stored as `[x, y, z, rx, ry, rz]`;
- SciPy `Rotation.from_euler("ZYZ", ..., degrees=True)` for robot-pose rotation
  conversion;
- OpenCV `cv2.calibrateHandEye(..., method=cv2.CALIB_HAND_EYE_PARK)` to compute
  the hand-eye transform.

The stored translation values are kept in millimeters. Runtime code converts
the translation part to meters after loading the file.

## Re-calibration

Regenerate and replace `T_gripper2camera.npy` whenever any of these change:

- camera mount position or angle;
- gripper/TCP definition;
- robot or end-effector hardware;
- camera intrinsics, stream profile, or camera device.

Use the tutorial scripts to regenerate the artifact:

1. Prepare the chessboard and fix it inside the Doosan robot workspace.
2. Bring up the real Doosan M0609 robot:

   ```bash
   ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real model:=m0609 host:=192.168.1.100
   ```

3. Switch robot mode as needed for teaching and remote control:

   ```bash
   ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "robot_mode: 0"
   ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "robot_mode: 1"
   ```

4. Install `v4l-utils` and identify the color camera `video*` device whose
   pixel format and colorspace match the depth camera:

   ```bash
   sudo apt install v4l-utils
   ```

5. Set the camera device number in `tutorial/data_recording.py`, then run it
   with the Doosan Python import path:

   ```bash
   export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/dsr_common2/lib/dsr_common2/imp
   cd src/macgyvbot_resources/calibration/tutorial
   python3 data_recording.py
   ```

   Press `q` in the camera window to save each image and robot pose. Collect at
   least 20 samples from different taught robot poses.

6. Confirm the capture output contains chessboard images and
   `data/calibrate_data.json`.
7. In `tutorial/handeye_calibration.py`, set:

   ```python
   checkerboard_size = (10, 7)
   square_size = 25
   ```

8. Run `tutorial/handeye_calibration.py` and confirm that it creates
   `T_gripper2camera.npy`.
9. Validate the generated artifact before robot motion. The PDF workflow uses
   `test.py` to click a camera pixel, convert it through depth and hand-eye
   calibration into the robot base frame, and run pick-and-place. Adjust
   Z-axis parameters such as `Z_OFFSET` only as part of validation.
10. After validation, replace this folder's `T_gripper2camera.npy` with the
    regenerated artifact.

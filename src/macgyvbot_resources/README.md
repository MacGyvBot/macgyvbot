# macgyvbot_resources

Shared non-code assets for migrated MacGyvBot packages.

## Scope

- `calibration/`
- `weights/`
- `weights/vlm/`

This package owns source-tree calibration and model asset locations under the
ROS workspace `src/` tree. Runtime packages should query
`macgyvbot_resources`; the repository root no longer owns installed model or
calibration assets. Printable ArUco marker PDFs are managed outside this
package; runtime code only depends on the marker IDs configured in
`macgyvbot_config.drawer`.

Runtime packages should use `macgyvbot_resources.calibration` and
`macgyvbot_resources.resources` helpers instead of duplicating package-share or
source-tree fallback lookup logic.

## YOLO RealSense Smoke Test

Place the YOLO weights at:

```text
src/macgyvbot_resources/weights/yolov11_best.pt
```

Then run the direct RealSense test:

```bash
python src/macgyvbot_resources/weights/test_yolo_realsense.py
```

The script opens the RealSense color stream at `640x480x30`, matching the
camera profile used in the root README, loads the YOLO weights, and displays
live detections. Press `q` to stop.

To run without a preview window:

```bash
python src/macgyvbot_resources/weights/test_yolo_realsense.py --no-display
```

If a ROS RealSense launch is already using the camera, stop it before running
this direct hardware test.

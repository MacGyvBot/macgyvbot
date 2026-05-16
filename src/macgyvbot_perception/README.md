# macgyvbot_perception

Perception package.

## Planned Scope

- YOLO object detection
- pick target resolution
- depth projection
- center/VLM grasp point selection
- human hand grasp detection
- locked mask and SAM helpers

## Current State

This package now owns the perception helpers and the hand grasp detection node
implementation. Shared constants and dataclasses are imported from
`macgyvbot_config` and `macgyvbot_domain`; model assets resolve through
`macgyvbot_resources`.

## Naming Rule

- `grasp_point/` is for robot grasp-point selection.
  - Use it for center/VLM grasp pixel selection and depth refinement.
  - It answers: "Where should the robot grasp this detected object?"
- `hand_tool_grasp/` is for human hand-tool grasp perception.
  - Use it for hand landmarks, tool ROI/mask contact, ML hand grasp
    classification, and overlay drawing.
  - It answers: "Is the user's hand grasping the tool?"

Keep ROS-facing names such as `hand_grasp_detection_node`,
`/hand_grasp_detection/annotated_image`, and `hand_grasp_model.pkl` stable unless
a deliberate interface migration is planned.

"""Prompts for the single-call VLM grasp method."""

from __future__ import annotations


def build_prompt(label, target_label, image_size):
    width, height = image_size
    return (
        "Choose one grasp center and wrist yaw for a two-finger parallel "
        "robot gripper.\n"
        "Return strict compact JSON only with keys: x_px, y_px, yaw_deg, confidence, reason.\n"
        f"Image size: width={width}, height={height}.\n"
        "Yaw definition:\n"
        "- yaw_deg is the additional robot wrist rotation from the current pose.\n"
        "- Positive yaw_deg means rotate the wrist left/counterclockwise in the image.\n"
        "- Negative yaw_deg means rotate the wrist right/clockwise in the image.\n"
        "- For long tools, rotate until the two gripper fingers close across the tool width.\n"
        "- If a long tool axis is about 45 degrees in the image, yaw_deg should be about 45 or -135.\n"
        "- If a long tool axis is about -45 degrees in the image, yaw_deg should be about -45 or 135.\n"
        "Constraints:\n"
        f"- x_px must be an integer in [0, {width - 1}]\n"
        f"- y_px must be an integer in [0, {height - 1}]\n"
        "- yaw_deg must be in [-180, 180]\n"
        "- choose a sturdy graspable region, not a sharp tip, blade, hole, or edge\n"
        "- use the visible object only; do not choose background\n"
        "- reason must be 8 words or fewer\n"
        "- no markdown, no code fence, no explanation outside JSON\n\n"
        f"Detected object label: {label}\n"
        f"Robot task target: {target_label}\n"
    )


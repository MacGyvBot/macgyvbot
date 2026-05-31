"""Prompts for the single-call VLM grasp method."""

from __future__ import annotations


def build_prompt(label, target_label, image_size):
    width, height = image_size
    return (
        "You are controlling a two-finger parallel robot gripper.\n\n"
        "Return strict compact JSON only with keys: x_px, y_px, yaw_deg, confidence, reason.\n"
        f"Image size is the cropped object image: width={width}, height={height}.\n\n"

        f"Detected object label: {label}\n"
        f"Robot task target: {target_label}\n\n"

        "Goal:\n"
        "- Choose one safe grasp point on the visible target object.\n"
        "- Estimate the additional wrist yaw needed from the current gripper pose.\n\n"

        "Coordinate rules:\n"
        "- x_px and y_px are cropped-image pixel coordinates.\n"
        f"- x_px must be an integer in [0, {width - 1}].\n"
        f"- y_px must be an integer in [0, {height - 1}].\n"
        "- The point must be on the visible object, not background.\n\n"

        "Grasp point rules:\n"
        "- Choose a sturdy graspable region.\n"
        "- Prefer a handle, shaft, or thick middle part.\n"
        "- Avoid sharp tips, blades, holes, edges, thin ends, and occluded regions.\n"
        "- For long tools, choose the center of the visible graspable handle or shaft.\n"
        "- Do not choose the geometric center if it is not graspable.\n\n"

        "Yaw rules:\n"
        "- yaw_deg is the additional wrist rotation from the current gripper pose.\n"
        "- The current gripper pose already grasps a horizontal tool correctly.\n"
        "- If the graspable part is horizontal in the image, yaw_deg must be near 0.\n"
        "- A nearly horizontal graspable part should have a small yaw near 0.\n"
        "- Estimate the real continuous tilt of the graspable part relative to horizontal.\n"
        "- Positive yaw_deg means rotate counterclockwise in the image.\n"
        "- Negative yaw_deg means rotate clockwise in the image.\n"
        "- Do not copy or snap to common angles.\n"
        "- Do not use 45, -45, 90, or -90 unless the image truly requires it.\n"
        "- For long tools, estimate yaw from the graspable handle or shaft axis only.\n"
        "- Because the gripper is 180-degree symmetric, choose the equivalent yaw closest to 0.\n"
        "- yaw_deg must be in [-180, 180].\n\n"

        "Output rules:\n"
        "- confidence must be a number from 0.0 to 1.0.\n"
        "- reason must be 8 words or fewer.\n"
        "- Return JSON only.\n"
        "- No markdown.\n"
        "- No code fence.\n"
        "- No explanation outside JSON.\n"
    )


"""Prompts for the grid-based VLM grasp method."""

from __future__ import annotations


def build_context_prompt(object_label: str, user_request: str | None) -> str:
    request_text = user_request or "Pick up the object in a natural way."
    return (
        "Describe the grasp-relevant geometry of the visible object for a "
        "two-finger robot gripper. Mention sturdy regions, unsafe regions, "
        "long axis direction, and likely grasp affordances. Keep it concise.\n\n"
        f"Object: {object_label}\n"
        f"Robot task: {request_text}\n"
    )


def build_grid_prompt(
    *,
    rows: int,
    cols: int,
    context: str,
    object_label: str,
    user_request: str | None,
) -> str:
    request_text = user_request or "Pick up the object in a natural way."
    return (
        "The image has a numbered grid overlay. Choose the single best grid "
        "cell for a two-finger robot gripper to grasp the object. Use the "
        "context to prefer affordance-aligned, sturdy, safe regions.\n"
        "Return JSON only with keys: row, col, cell, confidence, reason.\n"
        "Rows and columns are 1-indexed.\n\n"
        "Output must be a single JSON object. No extra text.\n"
        f"Object: {object_label}\n"
        f"Robot task: {request_text}\n"
        f"Context: {context}\n"
        f"Grid: {rows} rows x {cols} columns"
    )


def build_precise_prompt(
    *,
    image_size: tuple[int, int],
    object_label: str,
    user_request: str | None,
    context: str,
    coarse_point: tuple[float, float],
    coarse_yaw_deg: float,
) -> str:
    width, height = image_size
    request_text = user_request or "Pick up the object in a natural way."
    coarse_x = int(round(coarse_point[0]))
    coarse_y = int(round(coarse_point[1]))
    return (
        "Choose a precise grasp center and wrist yaw for a two-finger gripper.\n"
        "Return strict JSON only with keys: x_px, y_px, yaw_deg, confidence, reason.\n"
        f"Image size: width={width}, height={height}.\n"
        "Yaw definition:\n"
        "- yaw_deg is the additional robot wrist rotation from the current pose.\n"
        "- Positive yaw_deg means rotate the wrist left/counterclockwise in the image.\n"
        "- Negative yaw_deg means rotate the wrist right/clockwise in the image.\n"
        "- For long tools, rotate until the two gripper fingers close across the tool width.\n"
        "- If a long tool axis is about 45 degrees in the image, yaw_deg should be about 45 or -135.\n"
        "- If a long tool axis is about -45 degrees in the image, yaw_deg should be about -45 or 135.\n"
        "Constraints:\n"
        f"- x_px must be integer in [0, {width - 1}]\n"
        f"- y_px must be integer in [0, {height - 1}]\n"
        "- yaw_deg must be in [-180, 180]\n"
        "- choose a sturdy graspable region, avoid edges/holes/slippery tips\n"
        "- no markdown, no code fence, no explanation outside JSON\n\n"
        f"Object: {object_label}\n"
        f"Robot task: {request_text}\n"
        f"Context: {context}\n"
        f"Coarse point hint: x={coarse_x}, y={coarse_y}\n"
        f"Weak yaw hint from coarse geometry(deg): {coarse_yaw_deg:.2f}. "
        "Override it if it follows the object's long axis instead of the gripper closing direction.\n"
    )


def build_orientation_prompt(
    *,
    object_label: str,
    user_request: str | None,
    context: str,
    yaw_hint_deg: float | None = None,
) -> str:
    request_text = user_request or "Pick up the object in a natural way."
    hint = ""
    if yaw_hint_deg is not None:
        hint = f"Yaw hint from grasp point geometry: {yaw_hint_deg:.2f} degrees.\n"
    return (
        "Estimate the gripper end-effector orientation for this grasp.\n"
        "Return strict JSON only with keys: roll_deg, pitch_deg, yaw_deg, confidence, reason.\n"
        "If roll or pitch is uncertain, use 0.0. Keep yaw in [-180, 180].\n"
        "No markdown, no code fence, no explanation outside JSON.\n\n"
        f"Object: {object_label}\n"
        f"Robot task: {request_text}\n"
        f"Context: {context}\n"
        f"{hint}"
    )

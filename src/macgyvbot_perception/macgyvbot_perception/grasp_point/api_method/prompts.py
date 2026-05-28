"""Prompts for the API grasp method."""

from __future__ import annotations


def build_grasp_prompt(
    object_label: str,
    user_request: str | None,
    image_size: tuple[int, int],
    bbox_xyxy: tuple[int, int, int, int] | None = None,
) -> str:
    width, height = image_size
    request_text = user_request or "Pick up the object in a natural way."
    bbox_text = (
        f"Original image bbox xyxy: {bbox_xyxy}.\n"
        if bbox_xyxy is not None
        else ""
    )
    return (
        "Select one grasp pose for a robot two-finger parallel gripper. "
        "The image is a crop of one detected tool or object. Choose the "
        "best grasp-center pixel inside the crop and estimate end-effector "
        "orientation.\n\n"
        "Return strict JSON only. No markdown, code fences, or extra text. "
        "Schema: {\"x_px\": number, \"y_px\": number, "
        "\"roll_deg\": number, \"pitch_deg\": number, "
        "\"yaw_deg\": number, \"confidence\": number, "
        "\"reason\": string}.\n\n"
        f"Image size: width={width}, height={height}.\n"
        f"{bbox_text}"
        f"x_px range: 0 to {width - 1}. "
        f"y_px range: 0 to {height - 1}. "
        "roll_deg and pitch_deg must be degrees in [-180, 180]. "
        "yaw_deg is an additional wrist rotation from the current pose in [-90, 90]. "
        "The gripper fingers approach from the left and right sides of the viewed image "
        "and close horizontally along the image x-axis. "
        "confidence is [0, 1]. "
        "Prefer thick, rigid, stable regions near the balance point, such "
        "as handles or main bodies. Avoid blades, cutting edges, tips, "
        "holes, hinges, fragile parts, slippery ends, buttons, labels, and "
        "task-critical surfaces. For elongated objects, set yaw_deg so gripper "
        "fingers close across the shorter width. If roll or pitch is "
        "uncertain, use 0.0 but still estimate yaw.\n\n"
        f"Object: {object_label}\n"
        f"Robot task: {request_text}\n"
    )


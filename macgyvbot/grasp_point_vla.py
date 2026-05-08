"""VLA helper for the final grasp-state handoff.

This module is intentionally decoupled from ROS and MoveIt so it can be
plugged into the pick pipeline later without changing the control loop yet.

Intended flow:
1. YOLO detects the target object.
2. The robot moves to the detected x/y and descends only to a switch pose
   whose z stays above the object by a configured offset.
3. Right before closing the gripper, call the VLA with the latest camera view.
4. The VLA predicts one 7-DoF action step
   (dx, dy, dz, droll, dpitch, dyaw, gripper).
5. This module integrates that action with the current end-effector state and
   returns a recommended final grasp state for the robot arm.

The default implementation uses OpenVLA-style inference through Hugging Face.
OpenVLA predicts end-effector deltas rather than joint angles, so the returned
"robot arm state" is represented here as a Cartesian end-effector pose plus a
gripper command. Joint positions can be carried alongside as metadata.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation


DEFAULT_VLA_MODEL = "openvla/openvla-7b"
DEFAULT_UNNORM_KEY = "bridge_orig"
DEFAULT_LOCAL_MODEL_ROOT = Path(__file__).resolve().parents[1] / "models" / "vla"


@dataclass(frozen=True)
class Pose3D:
    """Cartesian end-effector pose in the robot base frame."""

    position_xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def rotation(self) -> Rotation:
        return Rotation.from_quat(self.quaternion_xyzw)

    def with_position(self, xyz: Sequence[float]) -> "Pose3D":
        xyz_tuple = tuple(float(v) for v in xyz)
        if len(xyz_tuple) != 3:
            raise ValueError("xyz must contain exactly 3 values.")
        return Pose3D(
            position_xyz=xyz_tuple,
            quaternion_xyzw=self.quaternion_xyzw,
        )


@dataclass(frozen=True)
class RobotArmState:
    """Robot state around the pre-grasp handoff."""

    ee_pose: Pose3D
    joint_positions: Mapping[str, float] | None = None
    gripper_opening: float | None = None


@dataclass(frozen=True)
class DetectedObjectContext:
    """Context from the detector/depth stage before the VLA takes over."""

    label: str
    base_position_xyz: tuple[float, float, float]
    bbox_xyxy: tuple[int, int, int, int] | None = None
    switch_offset_z_m: float = 0.03
    final_clearance_z_m: float = 0.005
    task_instruction: str | None = None

    def switch_position_xyz(self) -> tuple[float, float, float]:
        x, y, z = self.base_position_xyz
        return (x, y, z + self.switch_offset_z_m)


@dataclass(frozen=True)
class VLAAction:
    """One raw OpenVLA-style action step."""

    delta_xyz: tuple[float, float, float]
    delta_rpy: tuple[float, float, float]
    gripper: float | None
    full_action: tuple[float, ...]


@dataclass(frozen=True)
class GraspStateProposal:
    """Final proposal produced from the VLA action."""

    switch_state: RobotArmState
    recommended_state: RobotArmState
    object_context: DetectedObjectContext
    action: VLAAction
    prompt: str
    model_id: str
    notes: tuple[str, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class VLAConfig:
    """Runtime settings for the VLA adapter."""

    model_id: str = DEFAULT_VLA_MODEL
    local_model_root: Path = DEFAULT_LOCAL_MODEL_ROOT
    unnorm_key: str | None = DEFAULT_UNNORM_KEY
    use_4bit: bool = True
    trust_remote_code: bool = True
    highlight_bbox: bool = True
    position_delta_frame: str = "world"
    rotation_delta_frame: str = "tool"
    gripper_close_threshold: float = 0.0
    clamp_xy_radius_m: float | None = 0.10
    max_descent_from_switch_m: float | None = 0.08


class VLAStateModel:
    """Lazy-loading VLA wrapper that proposes a final grasp state.

    The default path is tuned for OpenVLA-style Hugging Face checkpoints. This
    keeps the model-specific logic in one place so the robot node can later call
    a simple `propose_grasp_state(...)` method near the grasp handoff.
    """

    def __init__(self, config: VLAConfig | None = None):
        self.config = config or VLAConfig()
        self.processor = None
        self.model = None
        self.torch = None
        self.device = None
        self.torch_dtype = None

    def load(self):
        """Load the VLA and processor only when needed."""
        if self.model is not None and self.processor is not None:
            return

        model_source = self._resolve_local_model_source()
        self._ensure_runtime_dependencies()

        from transformers import AutoModelForVision2Seq, AutoProcessor

        self.processor = AutoProcessor.from_pretrained(
            model_source,
            trust_remote_code=self.config.trust_remote_code,
            local_files_only=True,
        )

        has_accelerate = importlib.util.find_spec("accelerate") is not None
        has_bitsandbytes = importlib.util.find_spec("bitsandbytes") is not None
        device_map = "auto" if self.device == "cuda" and has_accelerate else None
        quant_config = self._make_quantization_config(has_bitsandbytes)

        model_kwargs = {
            "torch_dtype": self.torch_dtype,
            "low_cpu_mem_usage": True,
            "trust_remote_code": self.config.trust_remote_code,
            "local_files_only": True,
        }
        if device_map is not None:
            model_kwargs["device_map"] = device_map
        if quant_config is not None:
            model_kwargs["quantization_config"] = quant_config

        flash_attn_available = importlib.util.find_spec("flash_attn") is not None
        if flash_attn_available and self.device == "cuda":
            model_kwargs["attn_implementation"] = "flash_attention_2"

        self.model = AutoModelForVision2Seq.from_pretrained(
            model_source,
            **model_kwargs,
        )

        if device_map is None and quant_config is None:
            self.model.to(self.device)

        self.model.eval()

    def _resolve_local_model_source(self) -> str:
        local_dir = self.config.local_model_root / self.config.model_id.replace(
            "/",
            "__",
        )
        if (local_dir / "config.json").exists():
            return str(local_dir)

        raise FileNotFoundError(
            "로컬 VLA 가중치가 없거나 불완전합니다. "
            f"model_id={self.config.model_id}, expected_path={local_dir}. "
            "먼저 `python3 scripts/download_vla_weights.py`를 실행해주세요."
        )

    def unload(self):
        """Release processor/model memory after the grasp phase."""
        self.processor = None
        self.model = None

        if self.torch is not None and self.torch.cuda.is_available():
            self.torch.cuda.empty_cache()

    def predict_action(
        self,
        image: Image.Image | np.ndarray,
        instruction: str,
        *,
        object_bbox_xyxy: tuple[int, int, int, int] | None = None,
    ) -> VLAAction:
        """Run one VLA step and return a parsed 7-DoF action."""
        self.load()

        inference_image = self._prepare_image(
            image,
            object_bbox_xyxy=object_bbox_xyxy,
        )
        prompt = self._format_prompt(instruction)
        inputs = self.processor(prompt, inference_image)
        inputs = self._move_inputs_to_device(inputs)

        predict_kwargs: dict[str, Any] = {
            "do_sample": False,
        }
        if self.config.unnorm_key:
            predict_kwargs["unnorm_key"] = self.config.unnorm_key

        with self.torch.inference_mode():
            action = self.model.predict_action(**inputs, **predict_kwargs)

        return self._parse_action(action)

    def propose_grasp_state(
        self,
        image: Image.Image | np.ndarray,
        current_state: RobotArmState,
        object_context: DetectedObjectContext,
        *,
        instruction: str | None = None,
    ) -> GraspStateProposal:
        """Ask the VLA for the final pre-close arm state near the object.

        Parameters
        ----------
        image:
            Latest camera image from the grasp scene.
        current_state:
            End-effector state at the z-offset switch point, before the final
            grasp motion.
        object_context:
            YOLO/depth result already converted into the robot base frame.
        instruction:
            Optional override for the language instruction passed to the VLA.
        """
        switch_state = self._coerce_to_switch_state(current_state, object_context)
        prompt = instruction or self.build_grasp_instruction(
            switch_state=switch_state,
            object_context=object_context,
        )
        action = self.predict_action(
            image,
            prompt,
            object_bbox_xyxy=object_context.bbox_xyxy,
        )
        recommended_state, notes = self._integrate_action(
            switch_state,
            object_context,
            action,
        )

        return GraspStateProposal(
            switch_state=switch_state,
            recommended_state=recommended_state,
            object_context=object_context,
            action=action,
            prompt=prompt,
            model_id=self.config.model_id,
            notes=tuple(notes),
        )

    def build_grasp_instruction(
        self,
        *,
        switch_state: RobotArmState,
        object_context: DetectedObjectContext,
    ) -> str:
        """Build a VLA prompt for the last grasp-alignment step."""
        ee_x, ee_y, ee_z = switch_state.ee_pose.position_xyz
        obj_x, obj_y, obj_z = object_context.base_position_xyz
        task = object_context.task_instruction or f"grasp the {object_context.label}"

        return (
            f"Move from the current pre-grasp pose to a stable grasp pose for the "
            f"{object_context.label}. Keep the motion small and precise, align the "
            f"gripper with the object, and prepare to close only when the grasp is "
            f"well aligned. Task: {task}. "
            f"Current end-effector position in base frame: "
            f"x={ee_x:.3f}, y={ee_y:.3f}, z={ee_z:.3f}. "
            f"Detected object position in base frame: "
            f"x={obj_x:.3f}, y={obj_y:.3f}, z={obj_z:.3f}. "
            f"The robot is currently at a switch pose above the object by "
            f"{object_context.switch_offset_z_m:.3f} m."
        )

    def _coerce_to_switch_state(
        self,
        current_state: RobotArmState,
        object_context: DetectedObjectContext,
    ) -> RobotArmState:
        target_switch_xyz = object_context.switch_position_xyz()
        current_xyz = current_state.ee_pose.position_xyz

        use_current_xy = (
            abs(current_xyz[0] - target_switch_xyz[0]) < 1e-6
            and abs(current_xyz[1] - target_switch_xyz[1]) < 1e-6
        )

        if use_current_xy:
            switch_xyz = (
                current_xyz[0],
                current_xyz[1],
                max(current_xyz[2], target_switch_xyz[2]),
            )
        else:
            switch_xyz = target_switch_xyz

        switch_pose = current_state.ee_pose.with_position(switch_xyz)
        return RobotArmState(
            ee_pose=switch_pose,
            joint_positions=current_state.joint_positions,
            gripper_opening=current_state.gripper_opening,
        )

    def _integrate_action(
        self,
        switch_state: RobotArmState,
        object_context: DetectedObjectContext,
        action: VLAAction,
    ) -> tuple[RobotArmState, list[str]]:
        notes: list[str] = []
        current_pose = switch_state.ee_pose
        current_rotation = current_pose.rotation()

        delta_xyz = np.asarray(action.delta_xyz, dtype=float)
        if self.config.position_delta_frame == "tool":
            delta_xyz = current_rotation.apply(delta_xyz)
        elif self.config.position_delta_frame != "world":
            raise ValueError(
                "position_delta_frame must be either 'world' or 'tool'."
            )

        new_xyz = np.asarray(current_pose.position_xyz, dtype=float) + delta_xyz

        if self.config.rotation_delta_frame == "tool":
            delta_rotation = Rotation.from_euler("xyz", action.delta_rpy)
            new_rotation = current_rotation * delta_rotation
        elif self.config.rotation_delta_frame == "world":
            delta_rotation = Rotation.from_euler("xyz", action.delta_rpy)
            new_rotation = delta_rotation * current_rotation
        else:
            raise ValueError(
                "rotation_delta_frame must be either 'world' or 'tool'."
            )

        object_xyz = np.asarray(object_context.base_position_xyz, dtype=float)
        switch_xyz = np.asarray(switch_state.ee_pose.position_xyz, dtype=float)

        if self.config.clamp_xy_radius_m is not None:
            xy_delta = new_xyz[:2] - object_xyz[:2]
            xy_distance = float(np.linalg.norm(xy_delta))
            if xy_distance > self.config.clamp_xy_radius_m:
                direction = xy_delta / xy_distance
                new_xyz[:2] = (
                    object_xyz[:2] + direction * self.config.clamp_xy_radius_m
                )
                notes.append(
                    "Clamped final XY pose to stay near the detected object."
                )

        min_final_z = object_xyz[2] + object_context.final_clearance_z_m
        if new_xyz[2] < min_final_z:
            new_xyz[2] = min_final_z
            notes.append(
                "Raised final Z to keep a small clearance above the detected grasp point."
            )

        if self.config.max_descent_from_switch_m is not None:
            min_allowed_z = switch_xyz[2] - self.config.max_descent_from_switch_m
            if new_xyz[2] < min_allowed_z:
                new_xyz[2] = min_allowed_z
                notes.append(
                    "Limited descent from the switch pose before closing the gripper."
                )

        gripper_opening = switch_state.gripper_opening
        if action.gripper is not None:
            gripper_opening = action.gripper

        pose = Pose3D(
            position_xyz=tuple(float(v) for v in new_xyz),
            quaternion_xyzw=tuple(float(v) for v in new_rotation.as_quat()),
        )
        state = RobotArmState(
            ee_pose=pose,
            joint_positions=switch_state.joint_positions,
            gripper_opening=gripper_opening,
        )
        return state, notes

    def should_close_gripper(self, action: VLAAction) -> bool:
        """Interpret the scalar gripper output as a close/open decision."""
        if action.gripper is None:
            return False
        return action.gripper <= self.config.gripper_close_threshold

    def _prepare_image(
        self,
        image: Image.Image | np.ndarray,
        *,
        object_bbox_xyxy: tuple[int, int, int, int] | None = None,
    ) -> Image.Image:
        pil_image = self._coerce_image(image)

        if self.config.highlight_bbox and object_bbox_xyxy is not None:
            pil_image = self._draw_bbox(pil_image, object_bbox_xyxy)

        return pil_image

    @staticmethod
    def _coerce_image(image: Image.Image | np.ndarray) -> Image.Image:
        if isinstance(image, Image.Image):
            return image.convert("RGB")

        if not isinstance(image, np.ndarray):
            raise TypeError("image must be a PIL image or a numpy array.")

        if image.ndim == 2:
            return Image.fromarray(image).convert("RGB")

        if image.ndim != 3 or image.shape[2] not in (3, 4):
            raise ValueError("numpy image must have shape HxW, HxWx3, or HxWx4.")

        if image.shape[2] == 4:
            rgb = image[:, :, :3][:, :, ::-1]
        else:
            rgb = image[:, :, ::-1]

        return Image.fromarray(rgb.astype(np.uint8), mode="RGB")

    @staticmethod
    def _draw_bbox(
        image: Image.Image,
        bbox_xyxy: tuple[int, int, int, int],
    ) -> Image.Image:
        overlay = image.copy()
        draw = ImageDraw.Draw(overlay)
        draw.rectangle(bbox_xyxy, outline=(255, 64, 64), width=4)
        return overlay

    @staticmethod
    def _format_prompt(instruction: str) -> str:
        stripped = instruction.strip()
        if stripped.startswith("In:") and stripped.endswith("Out:"):
            return stripped
        return f"In: What action should the robot take to {stripped}?\nOut:"

    def _parse_action(self, action_output: Any) -> VLAAction:
        array = np.asarray(action_output, dtype=float).reshape(-1)
        if array.size < 7:
            raise RuntimeError(
                "VLA action output must contain at least 7 values "
                "(dx, dy, dz, droll, dpitch, dyaw, gripper)."
            )

        return VLAAction(
            delta_xyz=tuple(float(v) for v in array[:3]),
            delta_rpy=tuple(float(v) for v in array[3:6]),
            gripper=float(array[6]),
            full_action=tuple(float(v) for v in array.tolist()),
        )

    def _ensure_runtime_dependencies(self):
        if self.torch is not None:
            return

        if importlib.util.find_spec("torch") is None:
            raise ImportError(
                "torch is required for grasp_point_vla.py. "
                "Install PyTorch before using the VLA module."
            )

        if importlib.util.find_spec("transformers") is None:
            raise ImportError(
                "transformers is required for grasp_point_vla.py. "
                "Install transformers before using the VLA module."
            )

        import torch

        self.torch = torch
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = self._select_dtype()

    def _select_dtype(self):
        if self.device != "cuda":
            return self.torch.float32

        if self.torch.cuda.is_bf16_supported():
            return self.torch.bfloat16

        return self.torch.float16

    def _make_quantization_config(self, has_bitsandbytes: bool):
        if not self.config.use_4bit or self.device != "cuda" or not has_bitsandbytes:
            return None

        from transformers import BitsAndBytesConfig

        return BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=self.torch_dtype,
            bnb_4bit_use_double_quant=True,
        )

    def _move_inputs_to_device(self, inputs):
        target_device = self._model_input_device()
        moved = {}

        for key, value in inputs.items():
            if not hasattr(value, "to"):
                moved[key] = value
                continue

            if self.torch.is_floating_point(value):
                moved[key] = value.to(target_device, dtype=self.torch_dtype)
            else:
                moved[key] = value.to(target_device)

        return moved

    def _model_input_device(self):
        if hasattr(self.model, "hf_device_map"):
            for device in self.model.hf_device_map.values():
                if isinstance(device, int):
                    return self.torch.device(f"cuda:{device}")
                if isinstance(device, str) and device not in ("cpu", "disk"):
                    return self.torch.device(device)

        try:
            return next(self.model.parameters()).device
        except StopIteration:
            return self.torch.device(self.device)

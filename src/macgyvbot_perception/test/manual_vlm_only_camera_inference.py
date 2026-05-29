#!/usr/bin/env python3
"""Run one VLM-only inference on the current camera frame.

Usage after the camera and workspace environment are running:

    source install/setup.bash
    python3 src/macgyvbot_perception/test/manual_vlm_only_camera_inference.py \
        --target-label screwdriver

This is a manual diagnostic helper, not a pytest test.
"""

from __future__ import annotations

import argparse
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image

from macgyvbot_config.topics import CAMERA_COLOR_TOPIC
from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_VLM_ONLY,
    GRASP_POINT_MODE_VLM_ONLY_QWEN3B,
    GRASP_POINT_MODE_VLM_ONLY_QWEN7B,
    GRASP_POINT_MODE_VLM_ONLY_SMOL,
    VLM_ONLY_MODEL_BY_MODE,
)
from macgyvbot_perception.grasp_point.vlm_grasp_point_selector import VLMModel
from macgyvbot_perception.grasp_point.vlm_only_grasp_point_selector import (
    VLMOnlyGraspPointSelector,
)

try:
    import torch
except ImportError:
    torch = None


class PrintLogger:
    def info(self, message):
        print(f"[INFO] {message}", flush=True)

    def warn(self, message):
        print(f"[WARN] {message}", flush=True)

    def error(self, message):
        print(f"[ERROR] {message}", flush=True)


class OneFrameSubscriber(Node):
    def __init__(self, color_topic):
        super().__init__("manual_vlm_only_camera_inference")
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_stamp = None
        self.create_subscription(Image, color_topic, self._color_cb, 10)

    def _color_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.latest_stamp = msg.header.stamp


def wait_for_camera_frame(node, timeout_sec):
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_image is not None:
            return node.latest_image.copy()
    return None


def build_prompt(args, image_size):
    if args.prompt:
        return args.prompt

    return VLMOnlyGraspPointSelector._build_prompt(
        label=args.target_label,
        target_label=args.target_label,
        image_size=image_size,
        sam_source=None,
    )


def run_debug_generate(model, image, prompt, logger):
    model.load()

    prepared = model._prepare_image(image)
    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": prepared},
                {"type": "text", "text": prompt},
            ],
        }
    ]
    inputs = model.processor.apply_chat_template(
        messages,
        add_generation_prompt=True,
        tokenize=True,
        processor_kwargs={
            "return_dict": True,
            "return_tensors": "pt",
        },
    )
    if isinstance(inputs, dict) and "input_ids" in inputs:
        prompt_len = int(inputs["input_ids"].shape[-1])
        logger.info(f"Debug prompt_len={prompt_len}")
    elif hasattr(inputs, "shape"):
        prompt_len = int(inputs.shape[-1])
        logger.info(f"Debug prompt_len={prompt_len}")
    else:
        prompt_len = 0
        logger.info(f"Debug prompt_len=unknown inputs_type={type(inputs)}")

    inputs = model._move_inputs_to_device(inputs)

    if torch is None:
        raise RuntimeError("torch is required for debug generate")

    start = time.monotonic()
    with torch.inference_mode():
        if isinstance(inputs, dict):
            generated_ids = model.model.generate(
                **inputs,
                max_new_tokens=model.max_new_tokens,
                do_sample=False,
            )
        else:
            generated_ids = model.model.generate(
                inputs,
                max_new_tokens=model.max_new_tokens,
                do_sample=False,
            )
    logger.info(f"Debug generate elapsed_sec={time.monotonic() - start:.3f}")
    logger.info(f"Debug generated_shape={tuple(generated_ids.shape)}")

    full_text = model.processor.batch_decode(
        generated_ids,
        skip_special_tokens=False,
    )[0]
    full_text_skip = model.processor.batch_decode(
        generated_ids,
        skip_special_tokens=True,
    )[0]
    sliced_ids = generated_ids[:, prompt_len:] if prompt_len else generated_ids
    sliced_text = model.processor.batch_decode(
        sliced_ids,
        skip_special_tokens=False,
    )[0]
    sliced_text_skip = model.processor.batch_decode(
        sliced_ids,
        skip_special_tokens=True,
    )[0]

    logger.info(f"Debug full_decode_chars={len(full_text)}")
    print("----- FULL DECODE special=false -----", flush=True)
    print(full_text, flush=True)
    print("----- FULL DECODE special=true -----", flush=True)
    print(full_text_skip, flush=True)
    print("----- SLICED DECODE special=false -----", flush=True)
    print(sliced_text, flush=True)
    print("----- SLICED DECODE special=true -----", flush=True)
    print(sliced_text_skip, flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run one local VLM-only inference on the current camera frame."
    )
    parser.add_argument("--color-topic", default=CAMERA_COLOR_TOPIC)
    parser.add_argument("--target-label", default="screwdriver")
    parser.add_argument(
        "--mode",
        choices=(
            GRASP_POINT_MODE_VLM_ONLY,
            GRASP_POINT_MODE_VLM_ONLY_SMOL,
            GRASP_POINT_MODE_VLM_ONLY_QWEN3B,
            GRASP_POINT_MODE_VLM_ONLY_QWEN7B,
        ),
        default=GRASP_POINT_MODE_VLM_ONLY_SMOL,
        help="Named VLM-only model mode.",
    )
    parser.add_argument(
        "--model-id",
        default="",
        help="Optional explicit model id/path. Overrides --mode.",
    )
    parser.add_argument("--max-new-tokens", type=int, default=48)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument(
        "--preview-sec",
        type=float,
        default=1.0,
        help="Show the captured camera frame for this many seconds before inference.",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Do not open an OpenCV preview window.",
    )
    parser.add_argument(
        "--prompt",
        default="",
        help="Optional custom prompt. Defaults to the runtime VLM-only prompt.",
    )
    parser.add_argument(
        "--debug-decode",
        action="store_true",
        help="Print full/sliced decode with and without special tokens.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    logger = PrintLogger()
    model_id = args.model_id or VLM_ONLY_MODEL_BY_MODE[args.mode]

    rclpy.init(args=None)
    node = OneFrameSubscriber(args.color_topic)
    try:
        logger.info(
            "Waiting for camera frame: "
            f"topic={args.color_topic}, timeout_sec={args.timeout_sec}"
        )
        bgr_image = wait_for_camera_frame(node, args.timeout_sec)
        if bgr_image is None:
            logger.error("No camera frame received before timeout.")
            return 1

        if not args.no_preview:
            cv2.imshow("MacGyvBot VLM-only camera frame", bgr_image)
            preview_ms = max(1, int(args.preview_sec * 1000.0))
            logger.info(
                "Showing camera preview before inference: "
                f"preview_sec={args.preview_sec}"
            )
            cv2.waitKey(preview_ms)

        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        image = PILImage.fromarray(rgb_image)
        prompt = build_prompt(args, image.size)

        logger.info(
            "Loading VLM model: "
            f"mode={args.mode}, model_id={model_id}, "
            f"max_new_tokens={args.max_new_tokens}, "
            f"image_size={image.size}"
        )
        model = VLMModel(
            model_id=model_id,
            max_new_tokens=args.max_new_tokens,
            logger=logger,
        )

        if args.debug_decode:
            run_debug_generate(model, image, prompt, logger)
            return 0

        start = time.monotonic()
        result = model.ask(image, prompt)
        elapsed = time.monotonic() - start

        logger.info(f"Inference done: elapsed_sec={elapsed:.3f}")
        logger.info(f"Parsed JSON: {result.data}")
        logger.info("Raw response:")
        print(result.text, flush=True)

        if not args.no_preview:
            logger.info("Press any key in the preview window to close.")
            cv2.waitKey(0)
        return 0
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

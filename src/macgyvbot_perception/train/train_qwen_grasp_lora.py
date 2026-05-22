"""Fine-tune Qwen2.5-VL for single-call grasp point prediction.

The training target mirrors the runtime VLM-only pipeline:

    image + VLMOnlyGraspPointSelector prompt -> JSON with x_px, y_px, yaw_deg

Expected CSV columns:

    image_path,x_px,y_px,yaw_deg

Optional CSV columns:

    confidence,reason,label,target_label,sam_source

Image paths may be absolute, relative to the CSV file, relative to the dataset
root, or relative to the repository root. Fine-tuned LoRA adapter weights are
saved by default under:

    src/macgyvbot_resources/weights/vlm/Qwen_finetuning/
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import sys
from dataclasses import dataclass
from pathlib import Path

from PIL import Image


DEFAULT_MODEL_ID = "Qwen/Qwen2.5-VL-3B-Instruct"
DEFAULT_DATASET_ROOT = (
    Path(__file__).resolve().parents[1] / "data" / "grasp_dataset"
)
DEFAULT_OUTPUT_DIR = (
    Path(__file__).resolve().parents[2]
    / "macgyvbot_resources"
    / "weights"
    / "vlm"
    / "Qwen_finetuning"
)


def add_workspace_packages_to_path() -> None:
    workspace_src = Path(__file__).resolve().parents[2]
    for package_dir in workspace_src.iterdir():
        if package_dir.is_dir() and str(package_dir) not in sys.path:
            sys.path.insert(0, str(package_dir))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="LoRA fine-tune Qwen2.5-VL for MacGyvBot grasp JSON output."
    )
    parser.add_argument("--csv", required=True, type=Path, help="Training CSV path.")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=DEFAULT_DATASET_ROOT,
        help="Root used to resolve relative image paths.",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL_ID,
        help=(
            "HF model id or local path. If this model was downloaded by the "
            "project helper, the local weights/vlm directory is preferred."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Where LoRA adapter checkpoints are saved.",
    )
    parser.add_argument("--epochs", type=float, default=3.0)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--grad-accum", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=2e-4)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--save-steps", type=int, default=100)
    parser.add_argument("--logging-steps", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--no-4bit",
        action="store_true",
        help="Disable 4-bit QLoRA loading.",
    )
    parser.add_argument(
        "--no-sam-prompt",
        action="store_true",
        help="Do not include the runtime SAM overlay instruction in prompts.",
    )
    return parser.parse_args()


def local_vlm_root() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "macgyvbot_resources"
        / "weights"
        / "vlm"
    )


def resolve_model_source(model_id: str) -> str:
    requested = Path(model_id).expanduser()
    if requested.exists():
        return str(requested)

    local_dir = local_vlm_root() / model_id.replace("/", "__")
    if local_dir.exists():
        return str(local_dir)

    return model_id


def resolve_image_path(
    raw_path: str,
    csv_path: Path,
    dataset_root: Path,
) -> Path:
    candidate = Path(raw_path).expanduser()
    repo_root = Path(__file__).resolve().parents[3]
    candidates = []

    if candidate.is_absolute():
        candidates.append(candidate)
    else:
        candidates.extend(
            [
                csv_path.parent / candidate,
                dataset_root / candidate,
                repo_root / candidate,
            ]
        )

    for path in candidates:
        if path.exists():
            return path.resolve()

    return candidates[0].resolve()


def first_present(row: dict[str, str], keys: tuple[str, ...]) -> str:
    for key in keys:
        value = row.get(key)
        if value is not None and str(value).strip():
            return str(value).strip()
    return ""


@dataclass(frozen=True)
class GraspExample:
    image_path: Path
    answer: str
    label: str
    target_label: str
    sam_source: str | None


class GraspCsvDataset:
    def __init__(
        self,
        csv_path: Path,
        dataset_root: Path,
        use_sam_prompt: bool,
        max_samples: int = 0,
    ) -> None:
        self.csv_path = csv_path.expanduser().resolve()
        self.dataset_root = dataset_root.expanduser().resolve()
        self.use_sam_prompt = use_sam_prompt
        self.examples = self._load_examples(max_samples=max_samples)

    def __len__(self) -> int:
        return len(self.examples)

    def __getitem__(self, index: int) -> GraspExample:
        return self.examples[index]

    def _load_examples(self, max_samples: int) -> list[GraspExample]:
        rows = []
        with self.csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows.append(row)

        examples = []
        for row in rows:
            image_value = first_present(
                row,
                ("image_path", "sam_crop_path", "crop_path", "image"),
            )
            if not image_value:
                raise ValueError("CSV row is missing image_path/sam_crop_path/crop_path.")

            answer = self._answer_from_row(row)
            image_path = resolve_image_path(
                image_value,
                csv_path=self.csv_path,
                dataset_root=self.dataset_root,
            )
            if not image_path.exists():
                raise FileNotFoundError(f"Image not found: {image_path}")

            sam_source = first_present(row, ("sam_source",))
            if not sam_source and "sam" in image_path.name.lower():
                sam_source = "SAM_BBOX_PROMPT"
            if not self.use_sam_prompt:
                sam_source = ""

            examples.append(
                GraspExample(
                    image_path=image_path,
                    answer=answer,
                    label=first_present(row, ("label", "object_label")) or "object",
                    target_label=first_present(row, ("target_label",)) or "object",
                    sam_source=sam_source or None,
                )
            )

            if max_samples > 0 and len(examples) >= max_samples:
                break

        if not examples:
            raise ValueError(f"No examples loaded from CSV: {self.csv_path}")
        return examples

    @staticmethod
    def _answer_from_row(row: dict[str, str]) -> str:
        x_px = int(round(float(first_present(row, ("x_px", "x", "u")))))
        y_px = int(round(float(first_present(row, ("y_px", "y", "v")))))
        yaw_deg = float(first_present(row, ("yaw_deg", "yaw", "angle_deg")))
        confidence_raw = first_present(row, ("confidence",))
        confidence = float(confidence_raw) if confidence_raw else 1.0
        reason = first_present(row, ("reason",)) or "teacher generated grasp point"

        return json.dumps(
            {
                "x_px": x_px,
                "y_px": y_px,
                "yaw_deg": yaw_deg,
                "confidence": confidence,
                "reason": reason,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )


class QwenGraspCollator:
    def __init__(self, processor) -> None:
        self.processor = processor

    def __call__(self, examples: list[GraspExample]) -> dict:
        import torch

        batch = []
        prompt_lengths = []
        images = []

        from macgyvbot_perception.grasp_point.vlm_only_grasp_point_selector import (
            VLMOnlyGraspPointSelector,
        )

        for example in examples:
            image = Image.open(example.image_path).convert("RGB")
            images.append(image)
            prompt = VLMOnlyGraspPointSelector._build_prompt(
                label=example.label,
                target_label=example.target_label,
                image_size=image.size,
                sam_source=example.sam_source,
            )
            user_message = {
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": prompt},
                ],
            }
            full_messages = [
                user_message,
                {
                    "role": "assistant",
                    "content": [{"type": "text", "text": example.answer}],
                },
            ]
            prompt_text = self.processor.apply_chat_template(
                [user_message],
                tokenize=False,
                add_generation_prompt=True,
            )
            full_text = self.processor.apply_chat_template(
                full_messages,
                tokenize=False,
                add_generation_prompt=False,
            )
            prompt_inputs = self.processor(
                text=[prompt_text],
                images=[image],
                return_tensors="pt",
            )
            prompt_lengths.append(prompt_inputs["input_ids"].shape[-1])
            batch.append(full_text)

        model_inputs = self.processor(
            text=batch,
            images=images,
            padding=True,
            return_tensors="pt",
        )
        labels = model_inputs["input_ids"].clone()
        labels[labels == self.processor.tokenizer.pad_token_id] = -100

        for row_index, prompt_len in enumerate(prompt_lengths):
            labels[row_index, :prompt_len] = -100

        model_inputs["labels"] = labels
        return {key: value if torch.is_tensor(value) else value for key, value in model_inputs.items()}


def main() -> int:
    add_workspace_packages_to_path()
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("train_qwen_grasp_lora")

    try:
        import torch
        from peft import LoraConfig, get_peft_model, prepare_model_for_kbit_training
        from transformers import (
            AutoProcessor,
            BitsAndBytesConfig,
            Trainer,
            TrainingArguments,
            set_seed,
        )

        try:
            from transformers import AutoModelForImageTextToText as AutoVLMModel
        except ImportError:
            from transformers import AutoModelForVision2Seq as AutoVLMModel
    except ImportError as exc:
        logger.error("Missing training dependency: %s", exc)
        logger.error(
            "Install training dependencies, for example: "
            "python -m pip install peft"
        )
        return 2

    set_seed(args.seed)
    model_source = resolve_model_source(args.model)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("Model source: %s", model_source)
    logger.info("Output dir: %s", output_dir)

    processor = AutoProcessor.from_pretrained(model_source)
    tokenizer = processor.tokenizer
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token

    quant_config = None
    if not args.no_4bit:
        quant_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.bfloat16
            if torch.cuda.is_available() and torch.cuda.is_bf16_supported()
            else torch.float16,
            bnb_4bit_use_double_quant=True,
        )

    model_kwargs = {
        "device_map": "auto" if torch.cuda.is_available() else None,
        "low_cpu_mem_usage": True,
        "attn_implementation": "sdpa",
    }
    if quant_config is not None:
        model_kwargs["quantization_config"] = quant_config
    else:
        model_kwargs["dtype"] = (
            torch.bfloat16
            if torch.cuda.is_available() and torch.cuda.is_bf16_supported()
            else torch.float16
            if torch.cuda.is_available()
            else torch.float32
        )

    model = AutoVLMModel.from_pretrained(model_source, **model_kwargs)
    if quant_config is not None:
        model = prepare_model_for_kbit_training(model)

    lora_config = LoraConfig(
        r=16,
        lora_alpha=32,
        lora_dropout=0.05,
        bias="none",
        task_type="CAUSAL_LM",
        target_modules=[
            "q_proj",
            "k_proj",
            "v_proj",
            "o_proj",
            "gate_proj",
            "up_proj",
            "down_proj",
        ],
    )
    model = get_peft_model(model, lora_config)
    model.print_trainable_parameters()

    train_dataset = GraspCsvDataset(
        csv_path=args.csv,
        dataset_root=args.dataset_root,
        use_sam_prompt=not args.no_sam_prompt,
        max_samples=args.max_samples,
    )
    logger.info("Loaded %d training example(s).", len(train_dataset))

    training_args = TrainingArguments(
        output_dir=str(output_dir),
        num_train_epochs=args.epochs,
        per_device_train_batch_size=args.batch_size,
        gradient_accumulation_steps=args.grad_accum,
        learning_rate=args.learning_rate,
        logging_steps=args.logging_steps,
        save_steps=args.save_steps,
        save_total_limit=3,
        bf16=torch.cuda.is_available() and torch.cuda.is_bf16_supported(),
        fp16=torch.cuda.is_available() and not torch.cuda.is_bf16_supported(),
        remove_unused_columns=False,
        report_to="none",
        dataloader_num_workers=0,
    )
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=train_dataset,
        data_collator=QwenGraspCollator(processor),
    )
    trainer.train()
    trainer.save_model(str(output_dir))
    processor.save_pretrained(str(output_dir))
    logger.info("Saved fine-tuned adapter and processor to: %s", output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())

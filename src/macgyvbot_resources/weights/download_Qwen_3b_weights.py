#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

from huggingface_hub import snapshot_download

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "macgyvbot_config"))

from macgyvbot_config.vlm import VLM_MODEL_QWEN3B


MODEL_ID = VLM_MODEL_QWEN3B
DEST_ROOT = Path(__file__).resolve().parent / "vlm"


def main() -> None:
    dest = DEST_ROOT / MODEL_ID.replace("/", "__")
    dest.mkdir(parents=True, exist_ok=True)
    snapshot_download(
        repo_id=MODEL_ID,
        local_dir=str(dest),
        local_dir_use_symlinks=False,
    )
    print(f"Downloaded: {MODEL_ID}")
    print(f"Path: {dest}")


if __name__ == "__main__":
    main()

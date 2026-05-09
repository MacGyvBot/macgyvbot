#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

from huggingface_hub import snapshot_download


MODEL_ID = "HuggingFaceTB/SmolVLM2-2.2B-Instruct"
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

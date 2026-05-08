#!/usr/bin/env python3
from __future__ import annotations

import os
from pathlib import Path

from huggingface_hub import snapshot_download


MODEL_ID = os.environ.get("VLA_MODEL_ID", "openvla/openvla-7b")
DEST_ROOT = Path(__file__).resolve().parents[1] / "models" / "vla"


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

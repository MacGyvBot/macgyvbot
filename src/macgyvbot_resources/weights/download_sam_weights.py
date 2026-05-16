#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import urllib.request
from pathlib import Path


DEST_ROOT = Path(__file__).resolve().parent

MOBILE_SAM = {
    "repo_id": "dhkim2810/MobileSAM",
    "filename": "mobile_sam.pt",
    "sha256": "6dbb90523a35330fedd7f1d3dfc66f995213d81b29a5ca8108dbcdd4e37d6c2f",
}

SAM_VIT_B = {
    "url": "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth",
    "filename": "sam_vit_b_01ec64.pth",
}


def main() -> None:
    args = parse_args()
    dest_dir = Path(args.dest_dir).expanduser().resolve()
    dest_dir.mkdir(parents=True, exist_ok=True)

    if args.model in {"mobile_sam", "all"}:
        download_mobile_sam(dest_dir, force=args.force)

    if args.model in {"sam_vit_b", "all"}:
        download_url_checkpoint(
            url=SAM_VIT_B["url"],
            dest=dest_dir / SAM_VIT_B["filename"],
            force=args.force,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Download optional SAM checkpoints for handoff mask locking."
    )
    parser.add_argument(
        "--model",
        choices=("mobile_sam", "sam_vit_b", "all"),
        default="mobile_sam",
        help="Checkpoint to download. Default: mobile_sam.",
    )
    parser.add_argument(
        "--dest-dir",
        default=str(DEST_ROOT),
        help="Directory where checkpoints will be placed. Default: this weights directory.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Redownload even when the destination file already exists.",
    )
    return parser.parse_args()


def download_mobile_sam(dest_dir: Path, force: bool) -> None:
    try:
        from huggingface_hub import hf_hub_download
    except ImportError as exc:
        raise SystemExit(
            "huggingface_hub is required for MobileSAM download. "
            "Install requirements.txt first."
        ) from exc

    dest = dest_dir / MOBILE_SAM["filename"]
    if dest.exists() and not force:
        verify_sha256(dest, MOBILE_SAM["sha256"])
        print(f"Already exists: {dest}")
        return

    cached_path = hf_hub_download(
        repo_id=MOBILE_SAM["repo_id"],
        filename=MOBILE_SAM["filename"],
        local_dir=str(dest_dir),
        local_dir_use_symlinks=False,
        force_download=force,
    )
    downloaded = Path(cached_path)
    if downloaded.resolve() != dest.resolve():
        shutil.copy2(downloaded, dest)

    verify_sha256(dest, MOBILE_SAM["sha256"])
    print(f"Downloaded MobileSAM: {dest}")


def download_url_checkpoint(url: str, dest: Path, force: bool) -> None:
    if dest.exists() and not force:
        print(f"Already exists: {dest}")
        return

    tmp = dest.with_suffix(dest.suffix + ".tmp")
    print(f"Downloading {url}")
    with urllib.request.urlopen(url) as response, tmp.open("wb") as output:
        shutil.copyfileobj(response, output)
    tmp.replace(dest)
    print(f"Downloaded SAM ViT-B: {dest}")


def verify_sha256(path: Path, expected_sha256: str) -> None:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)

    actual = digest.hexdigest()
    if actual != expected_sha256:
        print(
            f"ERROR: checksum mismatch for {path}\n"
            f"expected: {expected_sha256}\n"
            f"actual:   {actual}",
            file=sys.stderr,
        )
        raise SystemExit(1)


if __name__ == "__main__":
    main()

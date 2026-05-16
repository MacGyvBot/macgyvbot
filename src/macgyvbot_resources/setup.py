from glob import glob
import os
from pathlib import Path

from setuptools import find_packages, setup

package_name = "macgyvbot_resources"
package_dir = Path(__file__).resolve().parent


def gather_data_files(source_dir: str, install_prefix: str, patterns: list[str]):
    entries = []
    root = package_dir / source_dir
    if not root.exists():
        return entries

    files = []
    for pattern in patterns:
        files.extend(glob(str(root / pattern), recursive=True))
    files = sorted(
        os.path.relpath(path, package_dir)
        for path in files
        if Path(path).is_file()
    )
    if files:
        entries.append((install_prefix, files))
    return entries


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]
    + gather_data_files(
        "calibration",
        f"share/{package_name}/calibration",
        ["*.npy", "*.md"],
    )
    + gather_data_files(
        "weights",
        f"share/{package_name}/weights",
        ["*.pt", "*.pth", "*.pkl", "*.py", ".gitkeep"],
    )
    + gather_data_files(
        "weights/vlm",
        f"share/{package_name}/weights/vlm",
        ["**/*"],
    ),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MacGyvBot Team",
    maintainer_email="todo@example.com",
    description="Shared calibration and model resource files for MacGyvBot packages.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)

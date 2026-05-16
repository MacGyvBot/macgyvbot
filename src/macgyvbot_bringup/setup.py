from glob import glob
import os
from pathlib import Path

from setuptools import setup

package_name = "macgyvbot_bringup"
package_dir = Path(__file__).resolve().parent

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (
            f"share/{package_name}/config",
            [
                os.path.relpath(path, package_dir)
                for path in sorted((package_dir / "config").glob("*.yaml"))
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MacGyvBot Team",
    maintainer_email="todo@example.com",
    description="Launch and configuration package for MacGyvBot.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)

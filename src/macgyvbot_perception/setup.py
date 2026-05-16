from setuptools import find_packages, setup

package_name = "macgyvbot_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MacGyvBot Team",
    maintainer_email="todo@example.com",
    description="Camera, detection, depth projection, and hand-grasp perception for MacGyvBot.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hand_grasp_detection = macgyvbot_perception.hand_grasp_detection_node:main",
        ],
    },
)

from setuptools import setup

package_name = "macgyvbot_ui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MacGyvBot Team",
    maintainer_email="todo@example.com",
    description="Operator-facing UI adapters and GUI integration for MacGyvBot.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)

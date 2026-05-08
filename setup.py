from setuptools import find_packages, setup
from glob import glob
from pathlib import Path

package_name = 'macgyvbot'


def gather_data_files(root_dir: str, install_prefix: str):
    entries = []
    root = Path(root_dir)
    if not root.exists():
        return entries

    grouped = {}
    for path in root.rglob('*'):
        if not path.is_file():
            continue
        rel_parent = path.parent.relative_to(root)
        dest = Path(install_prefix) / rel_parent
        grouped.setdefault(str(dest), []).append(str(path))

    for dest, files in sorted(grouped.items()):
        entries.append((dest, sorted(files)))
    return entries

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/calibration', glob('calibration/*.npy') + glob('calibration/*.md')),
        ('share/' + package_name + '/weights', glob('weights/*.pt')),
        ('share/' + package_name + '/weights', glob('weights/*.py')),
    ] + gather_data_files('weights/vlm', 'share/' + package_name + '/weights/vlm'),
    install_requires=['setuptools', 'SpeechRecognition'],
    zip_safe=True,
    maintainer='ssu',
    maintainer_email='ssu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
            'console_scripts': [
                'macgyvbot = macgyvbot.nodes.macgyvbot_node:main',
                'hand_grasp_detection = macgyvbot.nodes.hand_grasp_detection_node:main',
                'command_input_node = macgyvbot.nodes.command_input_node:main',
            ],
    },
)

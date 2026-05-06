from setuptools import find_packages, setup
from glob import glob

package_name = 'macgyvbot'

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
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
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
            'macgyvbot = macgyvbot.macgyvbot:main',
            'hand_grasp_detection = macgyvbot.hand_grasp_detection_node:main',
            'stt_node = macgyvbot.stt_node:main',
        ],
    },
)

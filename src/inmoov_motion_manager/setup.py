from setuptools import setup
from glob import glob
import os

package_name = 'inmoov_motion_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='InMoov Project',
    maintainer_email='maintainer@example.com',
    description='RViz gesture manager and structured robot-intent bridge for InMoov.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motion_manager = inmoov_motion_manager.motion_manager:main',
            'intent_bridge = inmoov_motion_manager.intent_bridge:main',
        ],
    },
)

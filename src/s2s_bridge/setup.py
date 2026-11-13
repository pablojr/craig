from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's2s_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        # Ensures launch files are copied to the installation directory
        (os.path.join('share', package_name, 'launch'), glob('config/*.yaml', os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 Bridge mapping Speech-to-Speech metadata to hardware parameters.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Defines 'ros2 run s2s_bridge viseme_node' execution path
            'viseme_node = s2s_bridge.s2s_viseme_bridge_node:main',
        ],
    },
)

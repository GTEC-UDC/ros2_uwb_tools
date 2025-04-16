from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros2_uwb_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='valentinbarral',
    maintainer_email='valentin.barral@udc.es',
    description='ROS2 set of tools for working with UWB devices.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pozyx_ranging_reader = ros2_uwb_tools.pozyx_ranging_reader:main',
            'anchor_publisher = ros2_uwb_tools.anchor_publisher:main',
        ],
    },
)

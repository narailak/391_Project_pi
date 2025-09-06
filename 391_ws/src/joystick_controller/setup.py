from setuptools import setup
import os
from glob import glob

package_name = 'joystick_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if any
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include any config files if needed
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tpi5',
    maintainer_email='tpi5@example.com',
    description='Joystick controller package for gamepad input mapping',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = joystick_controller.joystick_node:main',
        ],
    },
)
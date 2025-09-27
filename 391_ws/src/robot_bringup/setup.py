
from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
            'launch/bringup_noservo.launch.py',
            'launch/bringup_all.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/joystick.yaml',
            'config/drive.yaml',
            'config/linear.yaml',
            'config/servo.yaml',
            'config/servo_switch.yaml',
            'config/gripper.yaml',
            'config/dril.yaml',
            'config/cam.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)

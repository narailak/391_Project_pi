from setuptools import setup
from glob import glob

package_name = 'servo_dril_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # ต้องมีไฟล์ servo_dig_controller/__init__.py
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ติดตั้งไฟล์ launch ทั้งโฟลเดอร์
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for servo position control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_dril_node = servo_dril_controller.servo_dril_node:main',
        ],
    },
)

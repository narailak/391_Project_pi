from setuptools import setup

package_name = 'dual_cam_mjpeg_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dual_cam_mjpeg.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Dual USB cameras → MJPEG over HTTP on localhost.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dual_cam_mjpeg_node = dual_cam_mjpeg_server.node:main',
        ],
    },
)

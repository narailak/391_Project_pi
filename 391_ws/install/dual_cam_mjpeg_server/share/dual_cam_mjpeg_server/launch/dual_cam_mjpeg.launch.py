from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cam0', default_value='1'),
        DeclareLaunchArgument('cam1', default_value='2'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('jpeg_q', default_value='60'),
        DeclareLaunchArgument('http_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('http_port', default_value='8080'),

        Node(
            package='dual_cam_mjpeg_server',
            executable='dual_cam_mjpeg_node',
            name='dual_cam_mjpeg_node',
            output='screen',
            parameters=[{
                'cam0_device': LaunchConfiguration('cam0'),
                'cam1_device': LaunchConfiguration('cam1'),
                'width':       LaunchConfiguration('width'),
                'height':      LaunchConfiguration('height'),
                'fps':         LaunchConfiguration('fps'),
                'jpeg_quality':LaunchConfiguration('jpeg_q'),
                'http_host':   LaunchConfiguration('http_host'),
                'http_port':   LaunchConfiguration('http_port'),
            }]
        )
    ])
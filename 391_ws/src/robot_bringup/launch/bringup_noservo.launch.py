from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os




def generate_launch_description():
arg_ns = DeclareLaunchArgument('ns', default_value='man')
arg_with_joystick = DeclareLaunchArgument('with_joystick', default_value='true')
arg_with_drive = DeclareLaunchArgument('with_drive', default_value='true')
arg_with_linear = DeclareLaunchArgument('with_linear', default_value='true')
arg_with_servo_sw = DeclareLaunchArgument('with_servo_switch', default_value='false')
arg_with_gripper = DeclareLaunchArgument('with_gripper', default_value='true')
arg_with_dril = DeclareLaunchArgument('with_dril', default_value='false')
arg_with_cam_send = DeclareLaunchArgument('with_cam_sender', default_value='true')
arg_with_cam_web = DeclareLaunchArgument('with_cam_web', default_value='true')


ns = LaunchConfiguration('ns')
with_joystick = LaunchConfiguration('with_joystick')
with_drive = LaunchConfiguration('with_drive')
with_linear = LaunchConfiguration('with_linear')
with_servo_sw = LaunchConfiguration('with_servo_switch')
with_gripper = LaunchConfiguration('with_gripper')
with_dril = LaunchConfiguration('with_dril')
with_cam_send = LaunchConfiguration('with_cam_sender')
with_cam_web = LaunchConfiguration('with_cam_web')


use_env_domain = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=os.environ.get('ROS_DOMAIN_ID', '96'))


pkg_share = get_package_share_directory('robot_bringup')
def cfg(name: str):
return os.path.join(pkg_share, 'config', name)


group = GroupAction([
PushRosNamespace(ns),


Node(package='joystick_controller', executable='joystick_node', name='joystick_node',
parameters=[cfg('joystick.yaml')], output='screen', condition=IfCondition(with_joystick)),


Node(package='drive_controller', executable='drive_node', name='drive_node',
parameters=[cfg('drive.yaml')], output='screen', condition=IfCondition(with_drive)),


Node(package='linear_controller', executable='linear_node', name='linear_node',
parameters=[cfg('linear.yaml')], output='screen', condition=IfCondition(with_linear)),


Node(package='servo_switch180_controller', executable='servo_switch180_node', name='servo_switch180_node',
parameters=[cfg('servo_switch.yaml')], output='screen', condition=IfCondition(with_servo_sw)),


Node(package='gripper_controller', executable='gripper_node', name='gripper_node',
parameters=[cfg('gripper.yaml')], output='screen', condition=IfCondition(with_gripper)),


Node(package='dril_controller', executable='dril_node', name='dril_node',
parameters=[cfg('dril.yaml')], output='screen', condition=IfCondition(with_dril)),


Node(package='cam_sensor', executable='dual_cam_send_node', name='dual_cam_send_node',
parameters=[cfg('cam.yaml')], output='screen', condition=IfCondition(with_cam_send)),


IncludeLaunchDescription(
PythonLaunchDescriptionSource(
os.path.join(get_package_share_directory('dual_cam_mjpeg_server'), 'launch', 'dual_cam_mjpeg.launch.py')
),
condition=IfCondition(with_cam_web),
),
])


return LaunchDescription([
use_env_domain,
arg_ns,
arg_with_joystick, arg_with_drive, arg_with_linear, arg_with_servo_sw,
arg_with_gripper, arg_with_dril, arg_with_cam_send, arg_with_cam_web,
group,
])


from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def _pkg_exists(name: str) -> bool:
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False

def generate_launch_description():
    ns = 'man'
    set_domain = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=os.environ.get('ROS_DOMAIN_ID', '96'))

    pkg_share = get_package_share_directory('robot_bringup')
    def cfg(name: str):
        return os.path.join(pkg_share, 'config', name)

    def make_group(context, *args, **kwargs):
        actions = [PushRosNamespace(ns)]

        # 0) joy driver (hardware -> /man/joy)
        if _pkg_exists('joy'):
            actions.append(Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.08}],
                output='screen',
                # บังคับ absolute /joy ให้กลายเป็นชื่อ relative "joy" => /man/joy
                remappings=[('/joy', 'joy')],
            ))
        else:
            actions.append(LogInfo(msg='[bringup] skip joy (not found)'))

        # 1) joystick_controller / joystick_node  (ส่ง cmd_vel -> cmd_move)
        if _pkg_exists('joystick_controller'):
            actions.append(Node(
                package='joystick_controller',
                executable='joystick_node',
                name='joystick_node',
                parameters=[cfg('joystick.yaml')],
                output='screen',
                remappings=[('cmd_vel', 'cmd_move')],
            ))
        else:
            actions.append(LogInfo(msg='[bringup] skip joystick_controller (not found)'))

        # 2) drive_controller / drive_node
        if _pkg_exists('drive_controller'):
            actions.append(Node(package='drive_controller', executable='drive_node', name='drive_node',
                                parameters=[cfg('drive.yaml')], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip drive_controller (not found)'))

        # 3) linear_controller / linear_node
        if _pkg_exists('linear_controller'):
            actions.append(Node(package='linear_controller', executable='linear_node', name='linear_node',
                                parameters=[cfg('linear.yaml')], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip linear_controller (not found)'))

        # 4) servo_switch180_controller / servo_switch180_node
        if _pkg_exists('servo_switch180_controller'):
            actions.append(Node(package='servo_switch180_controller', executable='servo_switch180_node', name='servo_switch180_node',
                                parameters=[cfg('servo_switch.yaml')], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip servo_switch180_controller (not found)'))

        # 5) gripper_controller / gripper_node
        if _pkg_exists('gripper_controller'):
            actions.append(Node(package='gripper_controller', executable='gripper_node', name='gripper_node',
                                parameters=[cfg('gripper.yaml')], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip gripper_controller (not found)'))

        # 6) dril_controller / dril_node
        if _pkg_exists('dril_controller'):
            actions.append(Node(package='dril_controller', executable='dril_node', name='dril_node',
                                parameters=[cfg('dril.yaml')], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip dril_controller (not found)'))

        # 7) servo_dril_controller / servo_dril_node
        if _pkg_exists('servo_dril_controller'):
            actions.append(Node(package='servo_dril_controller', executable='servo_dril_node', name='servo_dril_node',
                                parameters=[], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip servo_dril_controller (not found)'))

        # 8) tao_controller / tao_node
        if _pkg_exists('tao_controller'):
            actions.append(Node(package='tao_controller', executable='tao_node', name='tao_node',
                                parameters=[], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip tao_controller (not found)'))

        # 9) cam_sensor / dual_cam_send_node
        if _pkg_exists('cam_sensor'):
            actions.append(Node(package='cam_sensor', executable='dual_cam_send_node', name='dual_cam_send_node',
                                parameters=[cfg('cam.yaml')], output='screen'))
        else:
            actions.append(LogInfo(msg='[bringup] skip cam_sensor (not found)'))

        # 10) dual_cam_mjpeg_server (ผ่าน launch ของมันเอง)
        if _pkg_exists('dual_cam_mjpeg_server'):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('dual_cam_mjpeg_server'),
                                     'launch', 'dual_cam_mjpeg.launch.py')
                    )
                )
            )
        else:
            actions.append(LogInfo(msg='[bringup] skip dual_cam_mjpeg_server (not found)'))

        return [GroupAction(actions)]

    return LaunchDescription([set_domain, OpaqueFunction(function=make_group)])

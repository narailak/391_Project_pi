from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def _pkg_exists(name: str) -> bool:
    try:
        # will raise ValueError if not installed
        get_package_share_directory(name)
        return True
    except Exception:
        return False

def generate_launch_description():
    # ---- args ----
    arg_ns            = DeclareLaunchArgument('ns', default_value='man')
    arg_with_joystick = DeclareLaunchArgument('with_joystick',     default_value='true')
    arg_with_drive    = DeclareLaunchArgument('with_drive',        default_value='true')
    arg_with_linear   = DeclareLaunchArgument('with_linear',       default_value='true')
    arg_with_servo    = DeclareLaunchArgument('with_servo',        default_value='false')  # ปิด servo ปกติไว้ก่อน
    arg_with_servo_sw = DeclareLaunchArgument('with_servo_switch', default_value='true')
    arg_with_gripper  = DeclareLaunchArgument('with_gripper',      default_value='true')
    arg_with_dril     = DeclareLaunchArgument('with_dril',         default_value='true')
    # ตั้งค่าเริ่มต้นให้ cam เป็น false กัน error ถ้าไม่มีแพ็กเกจ
    arg_with_cam_send = DeclareLaunchArgument('with_cam_sender',   default_value='false')
    arg_with_cam_web  = DeclareLaunchArgument('with_cam_web',      default_value='false')

    ns            = LaunchConfiguration('ns')
    with_joystick = LaunchConfiguration('with_joystick')
    with_drive    = LaunchConfiguration('with_drive')
    with_linear   = LaunchConfiguration('with_linear')
    with_servo    = LaunchConfiguration('with_servo')
    with_servo_sw = LaunchConfiguration('with_servo_switch')
    with_gripper  = LaunchConfiguration('with_gripper')
    with_dril     = LaunchConfiguration('with_dril')
    with_cam_send = LaunchConfiguration('with_cam_sender')
    with_cam_web  = LaunchConfiguration('with_cam_web')

    # domain id default
    use_env_domain = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=os.environ.get('ROS_DOMAIN_ID', '96'))

    # paths
    pkg_share = get_package_share_directory('robot_bringup')
    def cfg(name: str):
        return os.path.join(pkg_share, 'config', name)

    # เราจะสร้าง group แบบไดนามิก เพื่อเช็คว่ามีแพ็กเกจ cam/web ก่อนค่อยใส่ Node
    def make_group(context, *args, **kwargs):
        actions = [
            PushRosNamespace(ns),

            # Joystick
            Node(
                package='joystick_controller', executable='joystick_node', name='joystick_node',
                parameters=[cfg('joystick.yaml')], output='screen',
                condition=IfCondition(with_joystick),
            ),

            # Drive
            Node(
                package='drive_controller', executable='drive_node', name='drive_node',
                parameters=[cfg('drive.yaml')], output='screen',
                condition=IfCondition(with_drive),
            ),

            # Linear
            Node(
                package='linear_controller', executable='linear_node', name='linear_node',
                parameters=[cfg('linear.yaml')], output='screen',
                condition=IfCondition(with_linear),
            ),

            # Servo (ปกติ) — ปิดด้วย arg
            Node(
                package='servo_controller', executable='servo_node', name='servo_node',
                parameters=[cfg('servo.yaml')], output='screen',
                condition=IfCondition(with_servo),
            ),

            # Servo 0/180
            Node(
                package='servo_switch180_controller', executable='servo_switch180_node', name='servo_switch180_node',
                parameters=[cfg('servo_switch.yaml')], output='screen',
                condition=IfCondition(with_servo_sw),
            ),

            # Gripper
            Node(
                package='gripper_controller', executable='gripper_node', name='gripper_node',
                parameters=[cfg('gripper.yaml')], output='screen',
                condition=IfCondition(with_gripper),
            ),

            # Drill
            Node(
                package='dril_controller', executable='dril_node', name='dril_node',
                parameters=[cfg('dril.yaml')], output='screen',
                condition=IfCondition(with_dril),
            ),
        ]

        # เพิ่มกล้องเฉพาะเมื่อมีแพ็กเกจ cam_sensor และเปิด with_cam_sender
        if _pkg_exists('cam_sensor'):
            actions.append(
                Node(
                    package='cam_sensor', executable='dual_cam_send_node', name='dual_cam_send_node',
                    parameters=[cfg('cam.yaml')], output='screen',
                    condition=IfCondition(with_cam_send),
                )
            )

        # เพิ่มเว็บเฉพาะเมื่อมีแพ็กเกจ dual_cam_mjpeg_server และเปิด with_cam_web
        if _pkg_exists('dual_cam_mjpeg_server'):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('dual_cam_mjpeg_server'), 'launch', 'dual_cam_mjpeg.launch.py')
                    ),
                    condition=IfCondition(with_cam_web),
                )
            )

        return [GroupAction(actions)]

    return LaunchDescription([
        use_env_domain,
        arg_ns,
        arg_with_joystick, arg_with_drive, arg_with_linear, arg_with_servo, arg_with_servo_sw,
        arg_with_gripper, arg_with_dril, arg_with_cam_send, arg_with_cam_web,
        OpaqueFunction(function=make_group),
    ])

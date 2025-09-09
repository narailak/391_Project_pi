#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 joystick mapper for Flydigi Dune Fox (XInput-like mapping)
- Subscribe: sensor_msgs/Joy on /man/joy
- Publish:   geometry_msgs/Twist on /cmd_vel
- Mapping is aligned with your original pygame script:
    LX=ax[0], LY=-ax[1], LT=(ax[2]+1)/2, RX=ax[3], RY=-ax[4], RT=(ax[5]+1)/2,
    DPad: ax[6], ax[7]  (fallback to buttons if axes missing)
    Buttons: A,B,X,Y, LB,RB, BACK,START, GUIDE, LS,RS
"""

import math
import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


def _safe_axis(axes, idx, default=0.0):
    return float(axes[idx]) if idx < len(axes) else float(default)


def _safe_button(buttons, idx, default=0):
    return int(buttons[idx]) if idx < len(buttons) else int(default)


class Gamepad:
    """โครงสร้างเก็บค่าจอย (normalized)"""
    def __init__(self):
        # Axes --------------------------------------------------------
        self.lx: float = 0.0   # 0: Left X
        self.ly: float = 0.0   # 1: Left Y (inverted)
        self.l2: float = 0.0   # 2: LT  (0..1)
        self.rx: float = 0.0   # 3: Right X
        self.ry: float = 0.0   # 4: Right Y (inverted)
        self.r2: float = 0.0   # 5: RT  (0..1)
        self.dpadLeftRight: float = 0.0  # 6: D-Pad LR (-1..1)
        self.dpadUpDown: float = 0.0     # 7: D-Pad UD (-1..1)

        # Buttons -----------------------------------------------------
        self.button_cross: float = 0.0      # 0: A
        self.button_circle: float = 0.0     # 1: B
        self.button_triangle: float = 0.0   # 2: X
        self.button_square: float = 0.0     # 3: Y
        self.l1: float = 0.0                # 4: LB
        self.r1: float = 0.0                # 5: RB
        self.button_share: float = 0.0      # 6: BACK
        self.button_option: float = 0.0     # 7: START
        self.button_logo: float = 0.0       # 8: GUIDE
        self.PressedLeftAnalog: float = 0.0 # 9:  LS (press)
        self.PressedRightAnalog: float = 0.0# 10: RS (press)

    def reset_toggles(self):
        """ตัวอย่างรีเซ็ตค่าแบบชั่วคราว (ถ้าต้องการใช้งานจริง ปรับให้เข้ากับ use-case)"""
        self.button_cross = 0.0
        self.button_circle = 0.0
        self.button_triangle = 0.0
        self.button_square = 0.0
        self.l1 = 0.0
        self.r1 = 0.0
        self.button_share = 0.0
        self.button_option = 0.0
        self.button_logo = 0.0
        self.PressedLeftAnalog = 0.0
        self.PressedRightAnalog = 0.0


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")

        # ---------- Parameters ----------
        self.declare_parameter("joy_topic", "/man/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 3.0)
        self.declare_parameter("debug_pub", True)  # publish all mapped values as Float32MultiArray

        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.debug_pub_enabled = bool(self.get_parameter("debug_pub").value)

        # ---------- Publishers ----------
        self.pub_move = self.create_publisher(
            Twist, cmd_vel_topic, qos_profile=qos.qos_profile_system_default
        )
        if self.debug_pub_enabled:
            self.pub_debug = self.create_publisher(
                Float32MultiArray, "~joy_debug".replace("~", self.get_fully_qualified_name()),
                qos_profile=qos.qos_profile_sensor_data
            )

        # ---------- Subscribers ----------
        self.create_subscription(
            Joy, joy_topic, self.joy_cb, qos_profile=qos.qos_profile_sensor_data
        )

        # ---------- State ----------
        self.gamepad = Gamepad()

        # ส่งคำสั่งความเร็วตามช่วงเวลา (10 ms)
        self.sent_data_timer = self.create_timer(0.01, self.send_cmd_vel)

        self.get_logger().info(
            f"[Joystick] Subscribing Joy from '{joy_topic}', publishing Twist to '{cmd_vel_topic}'"
        )

    def joy_cb(self, msg: Joy):
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        # Axes: ตามสคริปต์ pygame เดิม
        self.gamepad.lx = _safe_axis(axes, 0, 0.0)
        self.gamepad.ly = -_safe_axis(axes, 1, 0.0)  # invert Y ให้เหมือนเดิม
        self.gamepad.l2 = (_safe_axis(axes, 2, 0.0) + 1.0) / 2.0  # 0..1
        self.gamepad.rx = _safe_axis(axes, 3, 0.0)
        self.gamepad.ry = -_safe_axis(axes, 4, 0.0)  # invert Y
        self.gamepad.r2 = (_safe_axis(axes, 5, 0.0) + 1.0) / 2.0  # 0..1

        # D-Pad: ถ้าไม่มีใน axes ให้ลองอ่านจากปุ่มตามไดรเวอร์บางตัว
        if len(axes) >= 8:
            self.gamepad.dpadLeftRight = _safe_axis(axes, 6, 0.0)  # -1 (L), 0, +1 (R)
            self.gamepad.dpadUpDown = _safe_axis(axes, 7, 0.0)     # -1 (D), 0, +1 (U)
        else:
            # Fallback: ปุ่ม (ตัวอย่าง mapping ทั่วไป)
            left = _safe_button(buttons, 13, 0)
            right = _safe_button(buttons, 14, 0)
            up = _safe_button(buttons, 11, 0)
            down = _safe_button(buttons, 12, 0)
            self.gamepad.dpadLeftRight = float(right - left)  # -1..1
            self.gamepad.dpadUpDown = float(up - down)        # -1..1

        # Buttons
        self.gamepad.button_cross    = float(_safe_button(buttons, 0))  # A
        self.gamepad.button_circle   = float(_safe_button(buttons, 1))  # B
        self.gamepad.button_triangle = float(_safe_button(buttons, 2))  # X
        self.gamepad.button_square   = float(_safe_button(buttons, 3))  # Y
        self.gamepad.l1              = float(_safe_button(buttons, 4))  # LB
        self.gamepad.r1              = float(_safe_button(buttons, 5))  # RB
        self.gamepad.button_share    = float(_safe_button(buttons, 6))  # BACK
        self.gamepad.button_option   = float(_safe_button(buttons, 7))  # START
        self.gamepad.button_logo     = float(_safe_button(buttons, 8))  # GUIDE
        self.gamepad.PressedLeftAnalog  = float(_safe_button(buttons, 9))   # LS
        self.gamepad.PressedRightAnalog = float(_safe_button(buttons, 10))  # RS

        # ตัวอย่าง: กดปุ่ม LOGO เพื่อ reset ปุ่มชั่วคราว
        if self.gamepad.button_logo > 0.5:
            self.gamepad.reset_toggles()

        # Debug stream of all values
        if self.debug_pub_enabled:
            arr = Float32MultiArray()
            # ใส่เรียงเพื่อดูง่าย (axes แล้วตามด้วยปุ่ม)
            arr.data = [
                self.gamepad.lx, self.gamepad.ly, self.gamepad.l2,
                self.gamepad.rx, self.gamepad.ry, self.gamepad.r2,
                self.gamepad.dpadLeftRight, self.gamepad.dpadUpDown,
                self.gamepad.button_cross, self.gamepad.button_circle,
                self.gamepad.button_triangle, self.gamepad.button_square,
                self.gamepad.l1, self.gamepad.r1,
                self.gamepad.button_share, self.gamepad.button_option,
                self.gamepad.button_logo,
                self.gamepad.PressedLeftAnalog, self.gamepad.PressedRightAnalog,
            ]
            self.pub_debug.publish(arr)

    def send_cmd_vel(self):
        """แปลงค่าจอยเป็นความเร็วแล้ว publish /cmd_vel"""
        msg = Twist()
        # ตัวอย่างง่าย ๆ: เดินหน้าถอยหลังด้วย LY, หมุนด้วย RX
        msg.linear.x  = float(self.gamepad.ly * self.max_linear)
        msg.angular.z = float(self.gamepad.rx * self.max_angular)
        self.pub_move.publish(msg)


def main():
    rclpy.init()
    node = Joystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

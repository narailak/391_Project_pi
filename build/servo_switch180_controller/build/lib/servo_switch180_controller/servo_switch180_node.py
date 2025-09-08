#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: joy_servo
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/cmd_servo_switch180 (std_msgs/Int16)
- Behavior:   Press D-PAD LEFT to toggle between 0 and 180
- Debounce:   rising-edge + time-based
"""

import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


class JoyServo(Node):
    def __init__(self):
        super().__init__("joy_servo")

        # -------- Parameters --------
        self.declare_parameter('joy_topic', '/man/joy')
        self.declare_parameter('pub_topic', '/man/cmd_servo_switch180')
        self.declare_parameter('debounce_time', 0.30)         # วินาที
        self.declare_parameter('dpad_lr_axis_index', 6)       # แกนซ้าย/ขวา ของ D-Pad
        self.declare_parameter('dpad_left_button_index', 13)  # fallback: ปุ่ม D-Pad ซ้าย

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.debounce_time = float(self.get_parameter('debounce_time').value)
        self.dpad_lr_axis_index = int(self.get_parameter('dpad_lr_axis_index').value)
        self.dpad_left_button_index = int(self.get_parameter('dpad_left_button_index').value)

        # -------- State --------
        self.toggle_value = 0           # ค่าเริ่มต้น (0°)
        self.last_press_time = 0.0
        self.prev_left_active = False   # ใช้ตรวจจับ rising-edge

        # -------- Publisher --------
        self.pub = self.create_publisher(Int16, pub_topic, qos.qos_profile_system_default)

        # -------- Subscriber --------
        self.create_subscription(Joy, joy_topic, self.joy_callback,
                                 qos_profile=qos.qos_profile_sensor_data)

        self.get_logger().info(
            f"joy_servo started | pub='{pub_topic}', debounce={self.debounce_time:.2f}s"
        )

    def _safe_axis(self, axes, idx, default=0.0):
        return float(axes[idx]) if idx < len(axes) else float(default)

    def _safe_button(self, buttons, idx, default=0):
        return int(buttons[idx]) if idx < len(buttons) else int(default)

    def joy_callback(self, msg: Joy):
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        # อ่าน D-Pad ซ้าย: ใช้แกน (axis6 == -1) ถ้าไม่มีใช้ fallback ปุ่ม
        left_active_by_axis = (self._safe_axis(axes, self.dpad_lr_axis_index, 0.0) <= -0.9)
        left_active_by_btn  = (self._safe_button(buttons, self.dpad_left_button_index, 0) == 1)
        left_active = left_active_by_axis or left_active_by_btn

        now = time.time()
        if left_active and not self.prev_left_active and (now - self.last_press_time >= self.debounce_time):
            # Toggle ระหว่าง 0 ↔ 180
            self.toggle_value = 180 if self.toggle_value == 0 else 0
            self.last_press_time = now

            out = Int16()
            out.data = int(self.toggle_value)
            self.pub.publish(out)

            self.get_logger().info(f"D-PAD LEFT pressed → Published {out.data}")

        self.prev_left_active = left_active


def main():
    rclpy.init()
    node = JoyServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

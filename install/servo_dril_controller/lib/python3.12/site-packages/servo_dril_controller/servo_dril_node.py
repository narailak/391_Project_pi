#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: servo_dig_node
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/cmd_servo_dril (std_msgs/Int16)
- Behavior:   Press D-PAD LEFT to toggle between 0 and 90
- Debounce:   rising-edge + time-based
"""

import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


class JoyServoDig(Node):
    def __init__(self):
        super().__init__("servo_dril_node")

        # -------- Parameters --------
        self.declare_parameter('joy_topic', '/man/joy')
        self.declare_parameter('pub_topic', '/man/cmd_servo_dril')
        self.declare_parameter('debounce_time', 0.30)             # วินาที
        self.declare_parameter('dpad_lr_axis_index', 6)           # แกนซ้าย/ขวา ของ D-Pad (ส่วนมากคือ axis 6)
        self.declare_parameter('axis_left_value', 1.0)           # ค่าที่ถือว่าเป็น "ซ้าย" (+1.0 บางจอย)
        self.declare_parameter('axis_tolerance', 0.2)             # เผื่อคลาดเคลื่อน
        self.declare_parameter('dpad_left_button_index', 13)      # fallback: ปุ่ม D-Pad ซ้าย

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value

        self.debounce_time = float(self.get_parameter('debounce_time').value)
        self.dpad_lr_axis_index = int(self.get_parameter('dpad_lr_axis_index').value)
        self.axis_left_value = float(self.get_parameter('axis_left_value').value)
        self.axis_tolerance = float(self.get_parameter('axis_tolerance').value)
        self.dpad_left_button_index = int(self.get_parameter('dpad_left_button_index').value)

        # -------- State --------
        self.toggle_value = 0           # ค่าเริ่มต้น (0°)
        self.last_press_time = 0.0
        self.prev_left_active = False   # ใช้ตรวจจับ rising-edge
        self._last_dbg_time = 0.0

        # -------- Publisher --------
        self.pub = self.create_publisher(Int16, pub_topic, qos.qos_profile_system_default)

        # -------- Subscriber --------
        self.create_subscription(Joy, joy_topic, self.joy_callback,
                                 qos_profile=qos.qos_profile_sensor_data)

        self.get_logger().info(
            f"servo_dig_node started | pub='{pub_topic}', debounce={self.debounce_time:.2f}s | "
            f"axis6 left≈{self.axis_left_value} tol±{self.axis_tolerance} | fallback btn idx={self.dpad_left_button_index}"
        )

    def _safe_axis(self, axes, idx, default=0.0):
        return float(axes[idx]) if idx < len(axes) else float(default)

    def _safe_button(self, buttons, idx, default=0):
        return int(buttons[idx]) if idx < len(buttons) else int(default)

    def _is_axis_left(self, val: float) -> bool:
        # ถือว่าเป็น "ซ้าย" ถ้าแกนอยู่ใกล้ค่าที่กำหนด (เช่น -1.0 หรือ +1.0) ภายใน tolerance
        return abs(val - self.axis_left_value) <= self.axis_tolerance

    def joy_callback(self, msg: Joy):
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        axis_val = self._safe_axis(axes, self.dpad_lr_axis_index, 0.0)
        left_active_by_axis = self._is_axis_left(axis_val)
        left_active_by_btn  = (self._safe_button(buttons, self.dpad_left_button_index, 0) == 1)
        left_active = left_active_by_axis or left_active_by_btn

        # Debug throttle (1 Hz) เพื่อดู mapping จริง
        now = time.time()
        if now - self._last_dbg_time > 1.0:
            self._last_dbg_time = now
            self.get_logger().debug(
                f"[DBG] axis_idx={self.dpad_lr_axis_index} val={axis_val:.2f} "
                f"(left≈{self.axis_left_value}) | btn[{self.dpad_left_button_index}]="
                f"{self._safe_button(buttons, self.dpad_left_button_index, 0)}"
            )

        # Rising-edge + debounce
        if left_active and not self.prev_left_active and (now - self.last_press_time >= self.debounce_time):
            self.toggle_value = 90 if self.toggle_value == 0 else 0
            self.last_press_time = now

            out = Int16()
            out.data = int(self.toggle_value)
            self.pub.publish(out)

            self.get_logger().info(f"D-PAD LEFT → Published {out.data}")

        self.prev_left_active = left_active


def main():
    rclpy.init()
    node = JoyServoDig()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

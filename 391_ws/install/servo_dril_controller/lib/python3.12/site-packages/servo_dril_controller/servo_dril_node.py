#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: servo_dig_node
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/cmd_servo_dril (std_msgs/Int16)
- Behavior:   D-PAD LEFT → toggle between off_value and on_value
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

        self.declare_parameter('debounce_time', 0.30)          # วินาที
        self.declare_parameter('dpad_lr_axis_index', 6)        # แกนซ้าย/ขวา ของ D-Pad (มักเป็น axis 6)
        self.declare_parameter('axis_left_value', 1.0)         # ค่าที่ถือว่าเป็น "ซ้าย" (+1.0 หรือ -1.0 แล้วแต่จอย)
        self.declare_parameter('axis_tolerance', 0.2)          # เผื่อคลาดเคลื่อนแกน
        self.declare_parameter('dpad_left_button_index', 13)   # fallback: ปุ่ม D-Pad ซ้าย

        # ค่า toggle ที่ต้องการ (ปรับง่ายเหมือนตัวอย่าง gripper)
        self.declare_parameter('on_value', 120)                 # ค่าขณะ "เปิด"
        self.declare_parameter('off_value', 30)                 # ค่าขณะ "ปิด"

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value

        self.debounce_time = float(self.get_parameter('debounce_time').value)
        self.dpad_lr_axis_index = int(self.get_parameter('dpad_lr_axis_index').value)
        self.axis_left_value = float(self.get_parameter('axis_left_value').value)
        self.axis_tolerance = float(self.get_parameter('axis_tolerance').value)
        self.dpad_left_button_index = int(self.get_parameter('dpad_left_button_index').value)

        self.on_value  = int(self.get_parameter('on_value').value)
        self.off_value = int(self.get_parameter('off_value').value)

        # -------- State --------
        self.toggle_value = self.off_value
        self.last_press_time = 0.0
        self.prev_left_active = False   # ใช้ตรวจจับ rising-edge
        self._last_dbg_time = 0.0

        # -------- Publisher --------
        self.pub = self.create_publisher(Int16, pub_topic, qos.qos_profile_system_default)

        # -------- Subscriber --------
        self.create_subscription(Joy, joy_topic, self.joy_callback,
                                 qos_profile=qos.qos_profile_sensor_data)

        self.get_logger().info(
            f"[servo_dig_node] Sub='{joy_topic}', Pub='{pub_topic}', debounce={self.debounce_time:.2f}s | "
            f"axis{self.dpad_lr_axis_index} left≈{self.axis_left_value} tol±{self.axis_tolerance} | "
            f"fallback btn idx={self.dpad_left_button_index} | on={self.on_value}, off={self.off_value}"
        )

    @staticmethod
    def _safe_axis(axes, idx, default=0.0):
        return float(axes[idx]) if idx < len(axes) else float(default)

    @staticmethod
    def _safe_button(buttons, idx, default=0):
        return int(buttons[idx]) if idx < len(buttons) else int(default)

    def _is_axis_left(self, val: float) -> bool:
        # ถือว่า "ซ้าย" ถ้าแกนอยู่ใกล้ค่าเป้าหมาย (เช่น -1.0 หรือ +1.0 ตามจอย) ภายใน tolerance
        return abs(val - self.axis_left_value) <= self.axis_tolerance

    def joy_callback(self, msg: Joy):
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        axis_val = self._safe_axis(axes, self.dpad_lr_axis_index, 0.0)
        left_active_by_axis = self._is_axis_left(axis_val)
        left_active_by_btn  = (self._safe_button(buttons, self.dpad_left_button_index, 0) == 1)
        left_active = left_active_by_axis or left_active_by_btn

        # Debug throttle (1 Hz) — ช่วยดู mapping ของจอยจริง
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
            self.toggle_value = self.on_value if self.toggle_value == self.off_value else self.off_value
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

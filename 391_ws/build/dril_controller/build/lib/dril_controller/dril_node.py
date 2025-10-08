#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: joy_dril (toggle on button press)
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/moter_dril (std_msgs/Int16)
- Behavior:
    กด A ครั้งที่ 1  -> ส่ง off_value (ค่าเริ่ม: 0)
    กด A ครั้งที่ 2  -> ส่ง on_value  (ค่าเริ่ม: 100)
    กด A ครั้งที่ 3  -> ส่ง off_value (0)
    ... วนไปเรื่อยๆ
- Rising-edge detection + debounce ป้องกันการกดซ้ำเร็วเกิน
"""

import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


class JoyDril(Node):
    def __init__(self):
        super().__init__("joy_dril")

        # -------- Parameters --------
        self.declare_parameter('joy_topic', '/man/joy')
        self.declare_parameter('pub_topic', '/man/moter_dril')   # คงชื่อตามที่ใช้อยู่ (สะกด moter)
        self.declare_parameter('a_button_index', 0)               # ส่วนใหญ่ XInput: A = index 0
        self.declare_parameter('on_value', 100)
        self.declare_parameter('off_value', 0)
        self.declare_parameter('debounce_time', 0.15)             # วินาที
        self.declare_parameter('first_press_is_off', True)        # กดครั้งแรกให้ส่ง off_value (0)

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.a_idx = int(self.get_parameter('a_button_index').value)
        self.on_value = int(self.get_parameter('on_value').value)
        self.off_value = int(self.get_parameter('off_value').value)
        self.debounce_time = float(self.get_parameter('debounce_time').value)
        first_press_is_off = bool(self.get_parameter('first_press_is_off').value)

        # -------- State --------
        self.prev_btn = 0
        self.last_edge_time = 0.0
        # ถ้า True -> press ถัดไปจะส่ง off_value, ถ้า False -> ส่ง on_value
        self.next_send_is_off = True if first_press_is_off else False

        # -------- Publisher --------
        self.pub = self.create_publisher(Int16, pub_topic, qos.qos_profile_system_default)

        # -------- Subscriber --------
        self.create_subscription(
            Joy, joy_topic, self.joy_callback, qos_profile=qos.qos_profile_sensor_data
        )

        self.get_logger().info(
            f"joy_dril (toggle) started | sub='{joy_topic}', pub='{pub_topic}', "
            f"A_idx={self.a_idx}, on={self.on_value}, off={self.off_value}, "
            f"debounce={self.debounce_time}s, first_press_is_off={first_press_is_off}"
        )

    @staticmethod
    def _safe_button(buttons, idx, default=0):
        return int(buttons[idx]) if 0 <= idx < len(buttons) else int(default)

    def joy_callback(self, msg: Joy):
        # อ่านสถานะปุ่ม A ปัจจุบัน
        buttons = list(msg.buttons)
        curr = self._safe_button(buttons, self.a_idx, 0)

        # ตรวจจับ rising edge: 0 -> 1
        if self.prev_btn == 0 and curr == 1:
            now = time.monotonic()
            if now - self.last_edge_time >= self.debounce_time:
                # สลับค่าที่จะส่ง: ครั้งแรก (ตาม first_press_is_off) -> off_value
                out_val = self.off_value if self.next_send_is_off else self.on_value
                self._publish(out_val)
                # toggle สำหรับครั้งถัดไป
                self.next_send_is_off = not self.next_send_is_off
                self.last_edge_time = now

        # อัปเดตสถานะปุ่มก่อนหน้า
        self.prev_btn = curr

    def _publish(self, val: int):
        msg = Int16()
        msg.data = val
        self.pub.publish(msg)
        self.get_logger().info(f"Publish /man/moter_dril: {val}")


def main():
    rclpy.init()
    node = JoyDril()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

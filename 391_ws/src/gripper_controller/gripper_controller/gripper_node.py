#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: gripper_node
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/cmd_gripper (std_msgs/Int16)
- Behavior:   Press X to toggle between  and 0 (publish on rising-edge only)
- Debounce:   rising-edge + time-based
"""

import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


class GripperToggle(Node):
    def __init__(self):
        super().__init__("gripper_node")

        # -------- Parameters --------
        self.declare_parameter('joy_topic', '/man/joy')
        self.declare_parameter('pub_topic', '/man/cmd_gripper')
        self.declare_parameter('debounce_time', 0.30)     # วินาที
        self.declare_parameter('x_button_index', 2)       # XInput: X = index 2
        self.declare_parameter('on_value', 80)            # ค่าที่อยากส่งค่าแรก 
        self.declare_parameter('off_value', 0)            # ค่าที่อยากส่งอีกค่า

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.debounce_time = float(self.get_parameter('debounce_time').value)
        self.x_button_index = int(self.get_parameter('x_button_index').value)
        self.on_value  = int(self.get_parameter('on_value').value)
        self.off_value = int(self.get_parameter('off_value').value)

        # -------- State --------
        self.current_value = self.off_value
        self.last_press_time = 0.0
        self.prev_x_pressed = False  # สำหรับตรวจจับ rising-edge

        # -------- Publisher --------
        self.pub = self.create_publisher(Int16, pub_topic, qos.qos_profile_system_default)

        # -------- Subscriber --------
        self.create_subscription(
            Joy, joy_topic, self.joy_callback, qos_profile=qos.qos_profile_sensor_data
        )

        self.get_logger().info(
            f"[gripper_node] Sub='{joy_topic}', Pub='{pub_topic}', debounce={self.debounce_time:.2f}s, "
            f"X_idx={self.x_button_index}, on={self.on_value}, off={self.off_value}"
        )

    @staticmethod
    def _safe_button(buttons, idx, default=0):
        return int(buttons[idx]) if idx < len(buttons) else int(default)

    def joy_callback(self, msg: Joy):
        buttons = list(msg.buttons)
        x_pressed = (self._safe_button(buttons, self.x_button_index, 0) == 1)

        now = time.time()
        # rising-edge: ตอนนี้กด และ ก่อนหน้าไม่กด และผ่านช่วง debounce แล้ว
        if x_pressed and not self.prev_x_pressed and (now - self.last_press_time >= self.debounce_time):
            # toggle 40 switch on_value and off_value
            self.current_value = self.on_value if self.current_value == self.off_value else self.off_value
            self.last_press_time = now

            out = Int16()
            out.data = self.current_value
            self.pub.publish(out)
            self.get_logger().info(f"X pressed → Published {out.data}")

        # อัปเดตสถานะปุ่มก่อนหน้า
        self.prev_x_pressed = x_pressed


def main():
    rclpy.init()
    node = GripperToggle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

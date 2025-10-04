#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: joy_dril
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/moter_dril (std_msgs/Int16)
- Behavior:   Press A -> 50, else -> 0  (simple map; no toggle)
"""

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
        self.declare_parameter('pub_topic', '/man/moter_dril')
        self.declare_parameter('a_button_index', 0)   # ส่วนใหญ่ XInput: A = index 0
        self.declare_parameter('on_value', 100)
        self.declare_parameter('off_value', 0)
        self.declare_parameter('publish_on_change_only', True)  # ลดสแปมบนบัส

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.a_idx = int(self.get_parameter('a_button_index').value)
        self.on_value = int(self.get_parameter('on_value').value)
        self.off_value = int(self.get_parameter('off_value').value)
        self.change_only = bool(self.get_parameter('publish_on_change_only').value)

        # -------- State --------
        self.last_out = None

        # -------- Publisher --------
        self.pub = self.create_publisher(Int16, pub_topic, qos.qos_profile_system_default)

        # -------- Subscriber --------
        self.create_subscription(
            Joy, joy_topic, self.joy_callback, qos_profile=qos.qos_profile_sensor_data
        )

        self.get_logger().info(
            f"joy_dril started | sub='{joy_topic}', pub='{pub_topic}', A_idx={self.a_idx}, "
            f"on={self.on_value}, off={self.off_value}, change_only={self.change_only}"
        )

    @staticmethod
    def _safe_button(buttons, idx, default=0):
        return int(buttons[idx]) if 0 <= idx < len(buttons) else int(default)

    def joy_callback(self, msg: Joy):
        buttons = list(msg.buttons)
        a_pressed = (self._safe_button(buttons, self.a_idx, 0) == 1)

        out_val = self.on_value if a_pressed else self.off_value
        if self.change_only and self.last_out is not None and out_val == self.last_out:
            return

        out = Int16()
        out.data = out_val
        self.pub.publish(out)
        self.last_out = out_val


def main():
    rclpy.init()
    node = JoyDril()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

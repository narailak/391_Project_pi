#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: joy_servo
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/cmd_slap (std_msgs/Int16MultiArray)
- Press B to toggle between 500 and 1500.
- Debounce: rising-edge + time-based (configurable via 'debounce_time' param)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

class JoyServo(Node):
    def __init__(self):
        super().__init__("joy_servo")

        # ---- parameters ----
        # debounce_time: วินาทีที่กันการกดซ้ำเร็วเกินไป
        self.declare_parameter('debounce_time', 0.30)
        self.debounce_time = float(self.get_parameter('debounce_time').value)

        # index ของปุ่ม B (ส่วนมากจอยแบบ Xbox: A=0, B=1, X=2, Y=3)
        self.declare_parameter('button_b_index', 1)
        self.button_b_index = int(self.get_parameter('button_b_index').value)

        # ---- state ----
        self.current_value = 500
        self.last_press_time = 0.0
        self.prev_b_state = 0  # ใช้ตรวจจับขอบขาขึ้น (rising edge)

        # ---- pubs / subs ----
        self.pub = self.create_publisher(
            Int16MultiArray, "/man/cmd_slap", qos_profile=qos.qos_profile_system_default
        )
        self.create_subscription(
            Joy, "/man/joy", self.joy_callback, qos_profile=qos.qos_profile_system_default
        )

        self.get_logger().info(
            f"joy_servo started | debounce_time={self.debounce_time:.2f}s | button_b_index={self.button_b_index}"
        )

    def joy_callback(self, msg: Joy):
        # ป้องกัน index error ถ้าจำนวนปุ่มน้อย
        b = 0
        if self.button_b_index < len(msg.buttons):
            b = msg.buttons[self.button_b_index]

        now = time.time()

        # rising edge: จาก 0 -> 1 เท่านั้นจึงถือว่า "กดใหม่"
        is_rising_edge = (self.prev_b_state == 0 and b == 1)

        if is_rising_edge and (now - self.last_press_time >= self.debounce_time):
            # toggle ระหว่าง 500 <-> 1500
            self.current_value = 1500 if self.current_value == 500 else 500
            self.last_press_time = now

            out = Int16MultiArray()
            out.data = [self.current_value]
            self.pub.publish(out)
            self.get_logger().info(f"B pressed → Published {self.current_value}")

        # อัปเดตสถานะปุ่มครั้งก่อน
        self.prev_b_state = b

def main():
    rclpy.init()
    node = JoyServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

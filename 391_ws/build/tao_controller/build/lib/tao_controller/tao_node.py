#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: joy_tao
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/cmd_tao (std_msgs/Int16)
- Behavior:
    กดปุ่ม Y หนึ่งครั้ง -> มุมเพิ่มทีละ (360 / n) องศา วนครบกลับ 0 แล้วเริ่มใหม่
    ค่าที่ส่งเป็นองศาแบบจำนวนเต็ม (int) ปัดด้วย round()

- Parameters:
    n:                    int   (default: 10)  จำนวนขั้นต่อรอบ (อนาคตปรับได้)
    y_button_index:       int   (default: 3)  ดัชนีปุ่ม Y (สไตล์ Xbox: A=0,B=1,X=2,Y=3)
    debounce_time:        float (default: 0.25) วินาที
    publish_initial_zero: bool  (default: True) เริ่มต้นส่งค่า 0 หนึ่งครั้ง
"""

import rclpy
from rclpy.node import Node
from rclpy import qos
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
import math
import time


def _safe_button(buttons, idx):
    return int(buttons[idx]) if (0 <= idx < len(buttons)) else 0


class JoyTao(Node):
    def __init__(self):
        super().__init__("joy_tao")

        # ---------- Parameters ----------
        self.declare_parameter('n', 2) # จำนวน ที่เอาไปหาร 360
        self.declare_parameter('y_button_index', 3)      # Xbox layout: Y=3
        self.declare_parameter('debounce_time', 0.25)     # วินาที
        self.declare_parameter('publish_initial_zero', True)

        self.n = int(self.get_parameter('n').value)
        self.y_idx = int(self.get_parameter('y_button_index').value)
        self.debounce = float(self.get_parameter('debounce_time').value)
        self.publish_initial_zero = bool(self.get_parameter('publish_initial_zero').value)

        # step องศาต่อหนึ่งกด
        self.step_deg = 360.0 / max(1, self.n)

        # ---------- State ----------
        self.current_index = 0            # 0..(n-1)
        self.last_y_state = 0             # ปุ่ม Y เฟรมก่อนหน้า
        self.last_press_time = 0.0        # สำหรับ debounce

        # ---------- Pub/Sub ----------
        self.pub = self.create_publisher(
            Int16, "/man/cmd_tao", qos_profile=qos.qos_profile_system_default
        )
        self.create_subscription(
            Joy, "/man/joy", self.joy_callback, qos_profile=qos.qos_profile_sensor_data
        )

        # อนุญาตให้ปรับพารามิเตอร์ขณะรัน
        self.add_on_set_parameters_callback(self.on_param_change)

        # ส่งค่าเริ่มต้น 0 ถ้าต้องการ
        if self.publish_initial_zero:
            self.publish_angle(0)
            self.get_logger().info("joy_tao started -> publish 0 (initial)")

        self.get_logger().info(
            f"joy_tao started | n={self.n} step={self.step_deg:.4f} deg | "
            f"Y idx={self.y_idx} debounce={self.debounce}s"
        )

    # --- Parameter update at runtime ---
    def on_param_change(self, params):
        for p in params:
            if p.name == 'n' and p.type_ == p.TYPE_INTEGER:
                new_n = max(1, int(p.value))
                self.n = new_n
                self.step_deg = 360.0 / self.n
                # รีเซ็ต index เพื่อความชัดเจน (หรือคงไว้ก็ได้ตามต้องการ)
                self.current_index = 0
                self.publish_angle(0)
                self.get_logger().info(
                    f"[param] n -> {self.n} | step={self.step_deg:.4f} deg | reset to 0"
                )
            elif p.name == 'y_button_index' and p.type_ == p.TYPE_INTEGER:
                self.y_idx = int(p.value)
                self.get_logger().info(f"[param] y_button_index -> {self.y_idx}")
            elif p.name == 'debounce_time' and (p.type_ in (p.TYPE_DOUBLE, p.TYPE_INTEGER)):
                self.debounce = float(p.value)
                self.get_logger().info(f"[param] debounce_time -> {self.debounce}s")
            elif p.name == 'publish_initial_zero' and p.type_ == p.TYPE_BOOL:
                self.publish_initial_zero = bool(p.value)
                self.get_logger().info(f"[param] publish_initial_zero -> {self.publish_initial_zero}")
        return SetParametersResult(successful=True)

    # --- Joy callback ---
    def joy_callback(self, msg: Joy):
        y_now = _safe_button(msg.buttons, self.y_idx)
        t_now = time.monotonic()

        # ตรวจจับกดครั้งใหม่ (rising edge) + debounce
        if (self.last_y_state == 0) and (y_now == 1):
            if (t_now - self.last_press_time) >= self.debounce:
                self.last_press_time = t_now
                self.advance_step()

        self.last_y_state = y_now

    # --- Core behavior ---
    def advance_step(self):
        # เลื่อนไปขั้นถัดไป, วนเมื่อครบ n
        self.current_index = (self.current_index + 1) % max(1, self.n)
        angle = int(round(self.current_index * self.step_deg)) % 360
        self.publish_angle(angle)
        self.get_logger().info(f"Y pressed -> index={self.current_index}/{self.n-1} angle={angle} deg")

    def publish_angle(self, angle_deg_int: int):
        out = Int16()
        out.data = int(angle_deg_int)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JoyTao()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
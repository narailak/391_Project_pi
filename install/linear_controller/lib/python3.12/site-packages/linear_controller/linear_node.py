#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 node: joy_linear
- Subscribes: /man/joy (sensor_msgs/Joy)
- Publishes:  /man/cmd_linear (std_msgs/Int16)
- Behavior:
    D-PAD UP   -> publish 1
    D-PAD DOWN -> publish -1
    otherwise  -> publish 0

- Notes:
    * หลายจอยจะให้ D-Pad เป็นแกนแนวตั้งที่ index=7 (ขึ้น=+1, ลง=-1)
    * หากคอนโทรลเลอร์ของคุณส่ง D-Pad เป็นปุ่ม ให้ตั้งค่าพารามิเตอร์
      'dpad_up_button_index' และ 'dpad_down_button_index' ให้ตรงกับ layout
"""

import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


def _safe_axis(axes, idx, default=0.0):
    return float(axes[idx]) if (0 <= idx < len(axes)) else float(default)


def _safe_button(buttons, idx):
    return int(buttons[idx]) if (0 <= idx < len(buttons)) else 0


class JoyLinear(Node):
    def __init__(self):
        super().__init__("joy_linear")

        # ---------- Parameters ----------
        # ดัชนีแกนแนวตั้งของ D-Pad (ทั่วไปคือ 7 สำหรับจอยสไตล์ Xbox)
        self.declare_parameter('dpad_vertical_axis_index', 7)
        self.axis_v_idx = int(self.get_parameter('dpad_vertical_axis_index').value)

        # Deadzone สำหรับแกน D-Pad (ถ้า |axis| >= deadzone จึงถือว่ากด)
        self.declare_parameter('dpad_deadzone', 0.5)
        self.deadzone = float(self.get_parameter('dpad_deadzone').value)

        # Fallback: ดัชนีปุ่มสำหรับ D-Pad UP/DOWN
        # ถ้าไม่ได้ใช้ ให้ตั้งเป็น -1
        self.declare_parameter('dpad_up_button_index', -1)
        self.declare_parameter('dpad_down_button_index', -1)
        self.btn_up_idx = int(self.get_parameter('dpad_up_button_index').value)
        self.btn_dn_idx = int(self.get_parameter('dpad_down_button_index').value)

        # ---------- Pub/Sub ----------
        self.pub = self.create_publisher(
            Int16, "/man/cmd_linear", qos_profile=qos.qos_profile_system_default
        )
        self.create_subscription(
            Joy, "/man/joy", self.joy_callback, qos_profile=qos.qos_profile_sensor_data
        )

        self.get_logger().info(
            f"joy_linear started | axis_v_idx={self.axis_v_idx} deadzone={self.deadzone} "
            f"| btn_up_idx={self.btn_up_idx} btn_dn_idx={self.btn_dn_idx}"
        )

    def joy_callback(self, msg: Joy):
        val = 0

        # 1) พยายามอ่านจากแกนแนวตั้งของ D-Pad ก่อน (ขึ้นเป็นค่าบวก, ลงเป็นค่าลบ)
        v = _safe_axis(msg.axes, self.axis_v_idx, 0.0)
        if v >= self.deadzone:
            val = 1
        elif v <= -self.deadzone:
            val = -1
        else:
            # 2) ถ้าแกนไม่ชัดเจน ลอง fallback เป็นปุ่ม (ถ้ากำหนด index ไว้และมีปุ่มนั้น)
            up_pressed = _safe_button(msg.buttons, self.btn_up_idx) if self.btn_up_idx >= 0 else 0
            dn_pressed = _safe_button(msg.buttons, self.btn_dn_idx) if self.btn_dn_idx >= 0 else 0

            if up_pressed and not dn_pressed:
                val = 1
            elif dn_pressed and not up_pressed:
                val = -1
            else:
                val = 0

        # Publish ทุกเฟรมเพื่อให้ downstream ตอบสนองตามสถานะปัจจุบัน
        out = Int16()
        out.data = int(val)
        self.pub.publish(out)

    # (ถ้าต้องการลด traffic ให้ publish เฉพาะตอนค่าเปลี่ยน สามารถเก็บ state ล่าสุดแล้วเช็คก่อนส่ง)


def main():
    rclpy.init()
    node = JoyLinear()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

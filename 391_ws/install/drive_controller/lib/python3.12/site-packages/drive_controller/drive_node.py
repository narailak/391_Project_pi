#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray
import math
from typing import List, Optional

class Drive_node(Node):

    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    motor1Speed : float = 0.0
    motor2Speed : float = 0.0
    motor3Speed : float = 0.0
    motor4Speed : float = 0.0

    def __init__(self):
        super().__init__("Drive_node")

        # ------ motion state ------
        self.moveSpeed = 0.0
        self.turnSpeed = 0.0

        # ------ robot geom (ยังไม่ใช้ทั้งหมด) ------
        self.wheel_base = 0.2        # m (distance between L/R wheels)
        self.wheel_radius = 0.060    # m (for your old calc path)

        self.maxSpeed = 1023.0       # pwm (not used here)
        self.maxRPM = 150            # temp conversion factor
        self.max_linear_speed = 3.0  # m/s max

        self.motor1Speed = 0.0
        self.motor2Speed = 0.0

        self.yaw = 0.0
        self.yaw_setpoint = self.yaw

        # ------ Parameters (แก้ทีหลังได้) ------
        self.declare_parameter('wheel_radius_cm', 4.5)   # ตามที่สั่ง
        self.declare_parameter('ticks_per_rev', 2400)    # ดีฟอลต์ 600 * 4 (ควอดราเจอร์)
        self.declare_parameter('gear_ratio', 1.0)        # แก้ภายหลังได้

        self.wheel_radius_cm: float = float(self.get_parameter('wheel_radius_cm').value)
        self.ticks_per_rev: float = float(self.get_parameter('ticks_per_rev').value)
        self.gear_ratio: float = float(self.get_parameter('gear_ratio').value)

        # ------ encoder accumulation state ------
        # last_counts: เก็บค่า int16 ล่าสุดที่รับมา (ความยาว 4)
        # cum_counts:  สะสมจำนวนติ๊กของแต่ละล้อ (int64) หลังแก้ wrap แล้ว (absolute, ต่อเนื่อง)
        self.last_counts: Optional[List[int]] = None
        self.cum_counts: List[int] = [0, 0, 0, 0]

        # ⭐ baseline นับจากตอนกด LB ล่าสุด (หน่วยเป็น "ติ๊กสะสมแบบต่อเนื่อง" เช่นเดียวกับ cum_counts)
        self.base_counts: List[int] = [0, 0, 0, 0]

        # ===== Publishers =====
        self.send_robot_speed = self.create_publisher(
            Twist, "/cmd_vel", qos_profile=qos.qos_profile_system_default
        )

        # (คงไว้ แต่จะไม่ใช้รีเซ็ตเอ็นโค้ดเดอร์จริง เพื่อให้สอดคล้องกับโจทย์)
        self.reset_pub = self.create_publisher(
            Int16, "/reset/motor_feedback", qos_profile=qos.qos_profile_system_default
        )

        # ส่งระยะทางล้อทั้ง 4 (เซนติเมตร) — ระยะสัมพัทธ์จาก baseline ล่าสุด
        self.dist_pub = self.create_publisher(
            Float32MultiArray, "/motor_feedback/distance_cm", qos_profile=qos.qos_profile_system_default
        )

        # ===== Subscriptions =====
        self.create_subscription(
            Twist, '/man/cmd_move', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        # จอย (ปุ่ม LB) -> รีเซ็ต baseline ระยะ (ไม่รีเซ็ตเอ็นโค้ดเดอร์จริง)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_cb, qos_profile=qos.qos_profile_sensor_data
        )

        # รับเอ็นโค้ดเดอร์ (Int16MultiArray: [enc1, enc2, enc3, enc4])
        self.enc_sub = self.create_subscription(
            Int16MultiArray, '/motor_feedback/encoders', self.enc_cb, qos_profile=qos.qos_profile_sensor_data
        )

        # ===== Timers =====
        self.sent_data_timer = self.create_timer(0.03, self.sendData)

        # ===== Debounce LB =====
        self.LB_INDEX = 4
        self.lb_last_raw = 0
        self.lb_last_stable = 0
        self.lb_last_change_time = self.get_clock().now()
        self.lb_debounce_ms = 40.0
        self.last_published_value = None

    # ----------------- Helpers -----------------
    @staticmethod
    def _unwrap_delta(curr_i16: int, prev_i16: int) -> int:
        """
        คืนค่าความต่างแบบแก้ wrap สำหรับสัญญาณ 16 บิต signed (-32768..32767)
        delta = ((curr - prev + 32768) % 65536) - 32768
        """
        delta = (int(curr_i16) - int(prev_i16))
        delta = ((delta + 32768) % 65536) - 32768
        return int(delta)

    def _publish_distances_cm(self):
        """
        แปลงระยะ (cm) ของล้อทั้ง 4 จาก (cum_counts - base_counts) แล้ว publish
        mapping: [M1=ซ้ายหน้า, M2=ขวาหน้า, M3=ซ้ายหลัง, M4=ขวาหลัง]
        """
        eff_cpr = self.ticks_per_rev * self.gear_ratio
        if eff_cpr <= 0.0:
            self.get_logger().warn("ticks_per_rev * gear_ratio <= 0, skip distance publish")
            return

        circum_cm = 2.0 * math.pi * self.wheel_radius_cm
        distances = []
        for i in range(4):
            rel_counts = float(self.cum_counts[i] - self.base_counts[i])
            rev = rel_counts / float(eff_cpr)
            dist_cm = rev * circum_cm
            distances.append(float(dist_cm))

        msg = Float32MultiArray()
        msg.data = distances
        self.dist_pub.publish(msg)

    # ----------------- Callbacks -----------------
    def cmd_vel(self, msg: Twist):
        linear_vel = msg.linear.y       # forward/backward ตามที่ผู้ใช้ตั้ง
        angular_vel = msg.angular.z * 5 # scale ที่ผู้ใช้ต้องการ

        v_left  = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        rpm_left  = float(v_left * self.maxRPM)
        rpm_right = float(v_right * self.maxRPM)

        self.motor1Speed = rpm_left
        self.motor2Speed = rpm_right

    def joy_cb(self, msg: Joy):
        # Debounce LB
        lb_raw = 0
        if len(msg.buttons) > self.LB_INDEX:
            lb_raw = 1 if msg.buttons[self.LB_INDEX] > 0 else 0

        now = self.get_clock().now()
        if lb_raw != self.lb_last_raw:
            self.lb_last_raw = lb_raw
            self.lb_last_change_time = now
            return

        elapsed_ms = (now - self.lb_last_change_time).nanoseconds / 1e6
        if elapsed_ms < self.lb_debounce_ms:
            return

        if lb_raw != self.lb_last_stable:
            self.lb_last_stable = lb_raw
            value_to_publish = 1 if self.lb_last_stable == 1 else 0

            # ✅ ไม่สั่งรีเซ็ตเอ็นโค้ดเดอร์จริง — ใช้ baseline ฝั่ง Python เท่านั้น
            if value_to_publish == 1:
                # ตรึง baseline เป็นค่าปัจจุบันของ cum_counts
                self.base_counts = list(self.cum_counts)
                # ส่งระยะ 0 ทันที
                zero_msg = Float32MultiArray()
                zero_msg.data = [0.0, 0.0, 0.0, 0.0]
                self.dist_pub.publish(zero_msg)
                self.get_logger().info("[RESET] baseline set to current cum_counts; distance_cm -> [0,0,0,0]")

            # ถ้าต้องการบอกสถานะปุ่ม (ไม่ไปรีเซ็ตฝั่ง ESP32)
            self.last_published_value = value_to_publish
            # ถ้าอยากปิด log นี้ ก็ลบได้
            self.get_logger().info(f"[LB] pressed={value_to_publish}")

    def enc_cb(self, msg: Int16MultiArray):
        """
        รับค่าจาก /motor_feedback/encoders (Int16MultiArray, 4 ช่อง)
        - คลี่ส่วนต่าง (unwrap) จากค่าเดิม -> อัปเดต cum_counts (absolute ต่อเนื่อง)
        - คำนวณระยะ (cm) จาก (cum_counts - base_counts) แล้ว publish
        """
        data = list(msg.data)
        if len(data) < 4:
            self.get_logger().warn("encoders array length < 4, ignore")
            return

        # เฟรมแรก: ให้ cum_counts = ค่าปัจจุบัน (signed 16-bit) เพื่อให้ได้ระยะทันที
        if self.last_counts is None:
            self.last_counts = [int(data[0]), int(data[1]), int(data[2]), int(data[3])]
            self.cum_counts  = [int(data[0]), int(data[1]), int(data[2]), int(data[3])]
            # baseline เริ่มต้นคือศูนย์ (base_counts=[0,0,0,0]) -> ระยะแรกจะคิดจากค่าปัจจุบันทันที
            self._publish_distances_cm()
            return

        # เฟรมถัดๆ ไป: สะสม delta แบบแก้ wrap
        for i in range(4):
            curr = int(data[i])
            prev = int(self.last_counts[i])
            d = self._unwrap_delta(curr, prev)   # ส่วนต่างภายใต้ wrap 16bit
            self.cum_counts[i] += int(d)
            self.last_counts[i] = curr

        # คำนวณและส่งระยะสัมพัทธ์จาก baseline
        self._publish_distances_cm()

    def publish_reset_value(self, value: int):
        # คง method ไว้เผื่อใช้ในอนาคต แต่ไม่ได้เรียกใช้งานใน logic ใหม่นี้
        msg = Int16()
        msg.data = int(value)
        self.reset_pub.publish(msg)

    def sendData(self):
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)
        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()
    sub = Drive_node()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

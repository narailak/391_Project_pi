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
        # wheel_radius_cm: รัศมีล้อจริงที่ต้องการใช้คำนวณระยะ (หน่วยเซนติเมตร)
        # ticks_per_rev: จำนวนติ๊กต่อ "รอบล้อ 1 รอบ" หลังรวมควอดราเจอร์แล้ว
        # gear_ratio: อัตราทด (counts_per_rev_effective = ticks_per_rev * gear_ratio)
        self.declare_parameter('wheel_radius_cm', 4.5)   # ตามที่สั่ง
        self.declare_parameter('ticks_per_rev', 2400)    # ดีฟอลต์ 600 * 4 (ควอดราเจอร์)
        self.declare_parameter('gear_ratio', 1.0)        # แก้ภายหลังได้

        self.wheel_radius_cm: float = float(self.get_parameter('wheel_radius_cm').value)
        self.ticks_per_rev: float = float(self.get_parameter('ticks_per_rev').value)
        self.gear_ratio: float = float(self.get_parameter('gear_ratio').value)

        # ------ encoder accumulation state ------
        # last_counts: เก็บค่า int16 ล่าสุดที่รับมา (ความยาว 4)
        # cum_counts:  สะสมจำนวนติ๊กของแต่ละล้อ (int64) หลังแก้ wrap แล้ว
        self.last_counts: Optional[List[int]] = None
        self.cum_counts: List[int] = [0, 0, 0, 0]

        # ===== Publishers =====
        self.send_robot_speed = self.create_publisher(
            Twist, "/cmd_vel", qos_profile=qos.qos_profile_system_default
        )

        # ส่งรีเซ็ตเอ็นโค้ดเดอร์ (1 ตอนกด, 0 ตอนปล่อย) -> ESP32 ฟัง /reset/motor_feedback (std_msgs/Int16)
        self.reset_pub = self.create_publisher(
            Int16, "/reset/motor_feedback", qos_profile=qos.qos_profile_system_default
        )

        # ใหม่: ส่งระยะทางล้อทั้ง 4 (เซนติเมตร)
        self.dist_pub = self.create_publisher(
            Float32MultiArray, "/motor_feedback/distance_cm", qos_profile=qos.qos_profile_system_default
        )

        # ===== Subscriptions =====
        self.create_subscription(
            Twist, '/man/cmd_move', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        # จอย (ปุ่ม LB) -> ส่งรีเซ็ต
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_cb, qos_profile=qos.qos_profile_sensor_data
        )

        # ใหม่: รับเอ็นโค้ดเดอร์ (Int16MultiArray: [enc1, enc2, enc3, enc4])
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
        คำนวณระยะ (cm) ของล้อทั้ง 4 จาก cum_counts แล้ว publish เป็น Float32MultiArray
        mapping: [M1=ซ้ายหน้า, M2=ขวาหน้า, M3=ซ้ายหลัง, M4=ขวาหลัง]
        """
        eff_cpr = self.ticks_per_rev * self.gear_ratio
        if eff_cpr <= 0.0:
            # กันพังถ้าตั้งพารามิเตอร์ผิด
            self.get_logger().warn("ticks_per_rev * gear_ratio <= 0, skip distance publish")
            return

        circum_cm = 2.0 * math.pi * self.wheel_radius_cm
        distances = []
        for i in range(4):
            rev = float(self.cum_counts[i]) / float(eff_cpr)
            dist_cm = rev * circum_cm
            distances.append(float(dist_cm))

        msg = Float32MultiArray()
        msg.data = distances
        self.dist_pub.publish(msg)

    # ----------------- Callbacks -----------------
    def cmd_vel(self, msg: Twist):
        linear_vel = msg.linear.y       # forward/backward ตามที่ผู้ใช้ตั้ง
        angular_vel = msg.angular.z * 5 # scale ที่ผู้ใช้ต้องการ

        # simple diff kinematics -> RPM (optional debug)
        v_left  = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        rpm_left  = float(v_left * self.maxRPM)
        rpm_right = float(v_right * self.maxRPM)

        self.motor1Speed = rpm_left
        self.motor2Speed = rpm_right

        # debug print (จะเห็นในเทอร์มินัล)
        # print(f"Left Motor: {self.motor1Speed:.2f} RPM, Right Motor: {self.motor2Speed:.2f} RPM")

    def joy_cb(self, msg: Joy):
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

            if self.last_published_value != value_to_publish:
                self.last_published_value = value_to_publish
                self.publish_reset_value(value_to_publish)

                # ถ้าเราสั่งรีเซ็ตเอง ให้รีเซ็ตตัวสะสมฝั่งนี้ด้วยเพื่อซิงค์
                if value_to_publish == 1:
                    self.cum_counts = [0, 0, 0, 0]
                    self.last_counts = None
                    # ส่งระยะ 0 ทันที
                    zero_msg = Float32MultiArray()
                    zero_msg.data = [0.0, 0.0, 0.0, 0.0]
                    self.dist_pub.publish(zero_msg)
                    self.get_logger().info("[RESET] local cum_counts cleared & distance_cm -> [0,0,0,0]")

    def enc_cb(self, msg: Int16MultiArray):
        """
        รับค่าจาก /motor_feedback/encoders (Int16MultiArray, 4 ช่อง)
        คลี่ส่วนต่าง (unwrap) จากค่าเดิม แล้วสะสม cum_counts
        จากนั้นคำนวณระยะ (cm) และ publish ไป /motor_feedback/distance_cm
        """
        data = list(msg.data)
        if len(data) < 4:
            self.get_logger().warn("encoders array length < 4, ignore")
            return

        if self.last_counts is None:
            self.last_counts = [int(data[0]), int(data[1]), int(data[2]), int(data[3])]
            # ยังไม่มี delta ให้คำนวณ แต่สามารถปล่อยเฟรมแรกได้เป็น 0
            self._publish_distances_cm()
            return

        for i in range(4):
            curr = int(data[i])
            prev = int(self.last_counts[i])
            d = self._unwrap_delta(curr, prev)   # ส่วนต่างภายใต้ wrap 16bit
            self.cum_counts[i] += int(d)
            self.last_counts[i] = curr

        # คำนวณและส่งระยะ
        self._publish_distances_cm()

    def publish_reset_value(self, value: int):
        msg = Int16()
        msg.data = int(value)
        self.reset_pub.publish(msg)
        self.get_logger().info(f"[LB] publish reset value: {value}")

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

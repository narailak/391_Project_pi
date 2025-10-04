#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy import qos
import time
import math

class Drive_node(Node):

    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0

    def __init__(self):
        super().__init__("Drive_node")

        self.moveSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.wheel_base = 0.2      # distance between wheels (meters)
        self.wheel_radius = 0.060  # radius of wheels (meters)

        self.maxSpeed : int = 1023.0 # pwm
        self.maxRPM : int = 150
        self.max_linear_speed = 3.0  # m/s max
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0

        self.yaw : float = 0
        self.yaw_setpoint = self.yaw

        # ----- Publishers -----
        self.send_robot_speed = self.create_publisher(
            Twist, "/cmd_vel", qos_profile=qos.qos_profile_system_default
        )

        # ส่งสัญญาณรีเซ็ตระยะ (1 ตอนกด, 0 ตอนปล่อย) -> ESP32 ฟังที่ /reset/motor_feedback (std_msgs/Int16)
        self.reset_pub = self.create_publisher(
            Int16, "/reset/motor_feedback", qos_profile=qos.qos_profile_system_default
        )

        # ----- Subscriptions -----
        self.create_subscription(
            Twist, '/man/cmd_move', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        # อ่านจอย (ปุ่ม LB) เพื่อส่ง 1/0 แบบ debounce
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_cb, qos_profile=qos.qos_profile_sensor_data
        )

        # ----- Timers -----
        self.sent_data_timer = self.create_timer(0.03, self.sendData)

        # ----- Debounce / Edge tracking for LB -----
        self.LB_INDEX = 4          # ปุ่ม LB มักจะเป็น index 4 (XBox mapping)
        self.lb_last_raw = 0       # ค่าปัจจุบันล่าสุดที่อ่านได้ (0/1)
        self.lb_last_stable = 0    # ค่าที่ผ่าน debounce แล้ว
        self.lb_last_change_time = self.get_clock().now()
        self.lb_debounce_ms = 40.0 # หน่วงดีบาวน์ ~40ms
        self.last_published_value = None  # กันส่งค่าซ้ำ (1 ซ้ำ ๆ)

    # ----------------- Callbacks -----------------
    def cmd_vel(self, msg: Twist):
        linear_vel = msg.linear.y     # forward/backward
        angular_vel = msg.angular.z   # turning rate
        angular_vel = angular_vel * 5 # scaling ที่ผู้ใช้ตั้งไว้

        # Compute left and right wheel speeds (in m/s)
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert to motor speeds in RPM (optional)
        rpm_left = float(v_left * self.maxRPM)
        rpm_right = float(v_right * self.maxRPM)

        self.motor1Speed = rpm_left
        self.motor2Speed = rpm_right

        print(f"Left Motor: {self.motor1Speed:.2f} RPM, Right Motor: {self.motor2Speed:.2f} RPM")

    def joy_cb(self, msg: Joy):
        # อ่านสถานะปุ่ม LB (ถ้าไม่มี index นั้น ให้ถือว่าไม่กด)
        lb_raw = 0
        if len(msg.buttons) > self.LB_INDEX:
            lb_raw = 1 if msg.buttons[self.LB_INDEX] > 0 else 0

        # Debounce: รับค่าใหม่ แล้วรอให้คงที่เกิน lb_debounce_ms ก่อนถือว่าเปลี่ยนจริง
        now = self.get_clock().now()
        if lb_raw != self.lb_last_raw:
            self.lb_last_raw = lb_raw
            self.lb_last_change_time = now
            return  # ยังไม่คงที่พอ

        # เวลาเปลี่ยนล่าสุดนานพอหรือยัง
        elapsed_ms = (now - self.lb_last_change_time).nanoseconds / 1e6
        if elapsed_ms < self.lb_debounce_ms:
            return  # ยังไม่นิ่งพอ

        # ถึงจุดนี้ถือว่าค่า stable แล้ว
        if lb_raw != self.lb_last_stable:
            self.lb_last_stable = lb_raw
            # Edge-trigger publish: 1 ตอนเปลี่ยนเป็นกด, 0 ตอนปล่อย
            value_to_publish = 1 if self.lb_last_stable == 1 else 0

            # กันส่งซ้ำค่าเดิม
            if self.last_published_value != value_to_publish:
                self.last_published_value = value_to_publish
                self.publish_reset_value(value_to_publish)

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

#!/usr/bin/env python3


import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
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

        self.wheel_base = 0.2  # distance between wheels (meters)
        self.wheel_radius = 0.060  # radius of wheels (meters)

        self.maxSpeed : int = 1023.0 # pwm
        self.maxRPM : int = 150
        self.max_linear_speed = 3.0  # m/s max
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0


        self.yaw : float = 0
        self.yaw_setpoint = self.yaw

        self.send_robot_speed = self.create_publisher(
            Twist, "/cmd_vel", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/man/cmd_move', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )


        self.sent_data_timer = self.create_timer(0.03, self.sendData)

    
    

    def cmd_vel(self, msg):

        linear_vel = msg.linear.y   # forward/backward
        angular_vel = msg.angular.z # turning rate
        angular_vel = angular_vel * 5

        # Compute left and right wheel speeds (in m/s)
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert to motor speeds in RPM (optional)
        rpm_left = float(v_left * self.maxRPM)
        rpm_right = float(v_right * self.maxRPM)

        # Store or send these speeds to motor controller
        self.motor1Speed = rpm_left
        self.motor2Speed = rpm_right

        print(f"Left Motor: {self.motor1Speed:.2f} RPM, Right Motor: {self.motor2Speed:.2f} RPM")


        

            
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
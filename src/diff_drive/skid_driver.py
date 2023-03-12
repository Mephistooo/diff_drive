#!/usr/bin/env python3
import atexit
import board
import rospy
from geometry_msgs.msg import Twist
from math import sin, cos, pi
from adafruit_motorkit import MotorKit

# Wheel angles (in radians) for a mecanum wheel configuration
wheel_angles = [0, pi/2, pi, 3*pi/2]

# Wheel coefficients for calculating wheel speeds
wheel_coeffs = [
    [cos(angle + pi/4) for angle in wheel_angles],
    [sin(angle + pi/4) for angle in wheel_angles],
    [1] * len(wheel_angles)
]

motor_driver = MotorKit(i2c=board.I2C())

def _clip_speed(speed):
     return max(min(speed, 1), -1)

def set_speed(speed: list):
        motor_driver.motor1.throttle = _clip_speed(speed[0])
        motor_driver.motor2.throttle = _clip_speed(speed[1])
        motor_driver.motor3.throttle = _clip_speed(speed[3])
        motor_driver.motor4.throttle = _clip_speed(speed[2])

def stop():
        motor_driver.motor1.throttle = 0
        motor_driver.motor2.throttle = 0
        motor_driver.motor3.throttle = 0
        motor_driver.motor4.throttle = 0

def callback(data):
    # Get linear and angular velocities from Twist message
    linear = data.linear.x
    angular = data.angular.z

    # Calculate left and right wheel velocities for skid steer drive
    left = linear - angular
    right = linear + angular

    # Calculate mecanum wheel speeds from left and right wheel velocities
    speeds = [0, 0, 0, 0]
    for i in range(len(speeds)):
        for j in range(len(wheel_coeffs)):
            speeds[i] += wheel_coeffs[j][i] * (left if j < 2 else right) * wheel_coeffs[j][2]

    set_speed(speeds)

def listener():
    # Initialize node
    rospy.init_node('motor_controller_2')

    # Subscribe to cmd_vel topic
    rospy.Subscriber('/cmd_vel', Twist, callback)

    # Spin until node is shut down
    rospy.spin()

if __name__ == '__main__':
    atexit.register(stop)
    listener()

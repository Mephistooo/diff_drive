#!/usr/bin/python3
import rospy
import math
import _thread
import time
from typing import List
from geometry_msgs.msg import Twist
import board
from adafruit_motorkit import MotorKit
import atexit
from math import pi

class VelocityCommand():

    def __init__(self):
        self.min = 0.4
        self.max = 1.0
        rospy.init_node('diff_motor_controller')
        self.WHEEL_RADIUS = 0.038
        self.WHEEL_GAP = 0.18
        self.left_speed = 0
        self.right_speed = 0 
        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)
        self.max_rpm  = 10
        self.motor_driver = MotorKit(i2c=board.I2C())
        self.last_x = 0
        self.last_z = 0
        # self.linear_thershold = 0.4
        atexit.register(self.stop)
        


    def set_speed(self, left_speed: float, right_speed: float):
        self.motor_driver.motor1.throttle = left_speed
        self.motor_driver.motor2.throttle = right_speed
        self.motor_driver.motor3.throttle = left_speed
        self.motor_driver.motor4.throttle = right_speed

    def stop(self):
        self.motor_driver.motor1.throttle = 0
        self.motor_driver.motor2.throttle = 0
        self.motor_driver.motor3.throttle = 0
        self.motor_driver.motor4.throttle = 0 

    def run(self):
            """driver loop"""
            rate = rospy.Rate(self._rate)
            while not rospy.is_shutdown():
                delay = rospy.get_time() - self._last_received
                if delay < self._timeout:
                    rospy.loginfo('Setting Speed - FLE: {}, FRE: {}'.format(self.left_speed , self.right_speed))
                    self.set_speed(self.left_speed,self.right_speed)
                else:
                    self.Stop()
                #rate.sleep()

    def set_pwm(self, data: Twist): 
        self._last_received = rospy.get_time()

        linear = data.linear.x
        angular = data.angular.z

        if linear == self.last_x and angular == self.last_z:
            return
             
        self.last_x = linear
        self.last_z = angular

        # if angular == 0 and linear == 0:
        #     self.left_speed  = 0
        #     self.right_speed = 0
        #     return
        # elif linear == 0 :
        #     right_speed = angular * self.WHEEL_GAP / 2.0
        #     left_speed = -right_speed
        # elif abs(angular) < 3/180*pi :
        #     right_speed = left_speed = linear
        # else :
        #     left_speed = linear - angular * self.WHEEL_GAP / 2.0
        #     right_speed = linear + angular * self.WHEEL_GAP / 2.0

        # if left_speed != 0:
        #     left_speed = 0 
        # if right_speed < 0.1:
        #     right_speed = 0 
        # left_speed*= 100
        # right_speed*= 100
        # # prece = int(speed_percent / 255 * 100)
        # # speed = int(min(max(abs(speed_percent * 255), 0), 255))

        # left_speed_percent = (0.1 * left_speed/1.0)
        # right_speed_percent = (0.1 * right_speed/1.0)
        # # convert velocities to [-1,1]
        # max_speed = (self.max_rpm / 60.0) * 2.0 * pi * (self.WHEEL_RADIUS * 0.5)

        # # left_speed_percent = float(min(max(left_speed * 0.1), -0.4), 1)
        # # right_speed_percent = float(min(max(right_speed * 0.1),-0.4), 1)
        # rospy.loginfo('FLE: {}, FRE: {}'.format(left_speed , right_speed))
        linear_vel = data.linear.x
        angular_vel = data.angular.z
     
            # Calculate speeds for left and right motors
        left_speed = linear_vel - angular_vel 
        right_speed = linear_vel + angular_vel
        if abs(left_speed) < 0.4 :
            left_speed = 0
        if abs(right_speed() < 0.4:
            right_speed = 0
        # # # Clip speeds to be within -1 and 1
        self.left_speed = max(min(left_speed, 1), -1)
        self.right_speed = max(min(right_speed, 1), -1)
    
        # self.left_speed= -left_speed_percent if left_speed < 0 else left_speed_percent
        # self.right_speed = -right_speed_percent if right_speed < 0 else right_speed_percent
        # # self.motor_driver.motor1.throttle = self.left_speed
        # self.motor_driver.motor2.throttle = self.right_speed
        # self.motor_driver.motor3.throttle = self.left_speed
        # self.motor_driver.motor4.throttle = self.right_speed

    def start_listening(self):
        rospy.Subscriber('/cmd_vel', Twist, self.set_pwm)
        # rospy.spin()
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                self.set_speed(self.left_speed, self.right_speed)
            else:
                self.stop()
            # rate.sleep()

if __name__ == '__main__':
    velocity_command = VelocityCommand()
    # velocity_command.start_listening()
    try :
        velocity_command.start_listening()
    except rospy.ROSInterruptException:
        velocity_command.stop()
        pass

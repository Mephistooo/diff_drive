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

class VelocityCommand():

    def __init__(self):
        self.min = 0.4
        self.max = 1.0
        self.motor_driver = MotorKit(i2c=board.I2C())
        rospy.init_node('diff_motor_controller')
        self.WHEEL_RADIUS = 0.038
        self.WHEEL_GAP = 0.18
        self.speed = []
        atexit.register(self.stop)


    def map(value: float, from_min: int, from_max: int, to_min: int, to_max: int):
        """
            Utility method to nomralize provided values from a range to a specific range of integers provided
            :param value: float
            :param from_min: int
            :param from_max: int
            :param to_min: int
            :param to_max: int    
        """
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

   

    def set_pwm(self, data: Twist): 
        linear = data.linear.x
        angular = data.angular.z
        if angular == 0 and linear == 0:
            self.motor_driver.motor1.throttle = 0
            self.motor_driver.motor2.throttle = 0
            self.motor_driver.motor3.throttle = 0
            self.motor_driver.motor4.throttle = 0
            return
        # rotation 
        elif linear == 0 :
            right_speed = angular * self.WHEEL_GAP / 2.0
            left_speed = -right_speed
        # forward or backward
        elif angular == 0 :
            right_speed = left_speed = linear
        else :
            left_speed = linear - angular * self.WHEEL_GAP / 2.0
            right_speed = linear + angular * self.WHEEL_GAP / 2.0

        # prece = int(speed_percent / 255 * 100)
        # speed = int(min(max(abs(speed_percent * 255), 0), 255))

        # _left_speed_percent = (0.01 * left_speed/1.0)
        #  _right_speed_percent = (0.01 * right_speed/1.0)

        left_speed_percent = float(min(max(abs(left_speed * 1), 0.5), 1.0))
        right_speed_percent = float(min(max(abs(right_speed * 1), 0.5), 1.0))

        rospy.loginfo('FLE: {0}, FRE: {1}'.format(left_speed_percent , right_speed_percent))

        self.motor_driver.motor1.throttle = -left_speed_percent if left_speed < 0 else left_speed_percent
        self.motor_driver.motor2.throttle = -right_speed_percent if right_speed < 0 else right_speed_percent
        self.motor_driver.motor3.throttle = -left_speed_percent if left_speed < 0 else left_speed_percent 
        self.motor_driver.motor4.throttle = -right_speed_percent if right_speed < 0 else right_speed_percent 
    
    def stopAll(self):
        self.motor_driver.motor1.throttle = 0
        self.motor_driver.motor2.throttle = 0
        self.motor_driver.motor3.throttle = 0
        self.motor_driver.motor4.throttle = 0 
        
    def start_listening(self):
            rospy.Subscriber('/cmd_vel', Twist, self.set_pwm)
            rospy.spin()
            
    def stop(self):
        """Stop all movement."""
        self.motor_driver.motor1.throttle = 0
        self.motor_driver.motor2.throttle = 0
        self.motor_driver.motor3.throttle = 0
        self.motor_driver.motor4.throttle = 0 

if __name__ == '__main__':
    velocity_command = VelocityCommand()
    try :
        # _thread.start_new_thread(velocity_command.set_pwm, ())
        velocity_command.start_listening()
    except rospy.ROSInterruptException:
        velocity_command.stopAll()
        pass

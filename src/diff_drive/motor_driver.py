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
        atexit.register(self.stop)
        self.left_speed = 0
        self.right_speed = 0 
        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)

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
        rate = rospy.Rate(10)
        self._last_received = rospy.get_time()
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
        left_speed_percent = float(min(max(abs(left_speed * 1), 0.4), 1.0))
        right_speed_percent = float(min(max(abs(right_speed * 1), 0.4), 1.0))
        rospy.loginfo('FLE: {}, FRE: {}'.format(left_speed , right_speed))

        self.left_speed= -left_speed_percent if left_speed < 0 else left_speed_percent
        self.right_speed = -right_speed_percent if right_speed < 0 else right_speed_percent
        self.motor_driver.motor1.throttle = self.left_speed
        self.motor_driver.motor2.throttle = self.right_speed
        self.motor_driver.motor3.throttle = self.left_speed
        self.motor_driver.motor4.throttle = self.right_speed
        rate.sleep()

    def stopAll(self):
        self.motor_driver.motor1.throttle = 0
        self.motor_driver.motor2.throttle = 0
        self.motor_driver.motor3.throttle = 0
        self.motor_driver.motor4.throttle = 0 
    
    def set_speed(self):
        self.motor_driver.motor1.throttle = self.left_speed
        self.motor_driver.motor2.throttle = self.right_speed
        self.motor_driver.motor3.throttle = self.left_speed
        self.motor_driver.motor4.throttle = self.right_speed

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
    # velocity_command.start_listening()
    try :
        # _thread.start_new_thread(velocity_command.set_pwm, ())
        velocity_command.start_listening()
    except rospy.ROSInterruptException:
        velocity_command.stopAll()
        pass

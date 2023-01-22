import rospy
import math
import _thread
import time
from typing import List
from geometry_msgs.msg import Twist
import board
from adafruit_motorkit import MotorKit

class VelocityCommand():

    def __init__(self):
        self.min = 0.4
        self.max = 1.0
        self.motor_driver = MotorKit(i2c=board.I2C())
        rospy.init_node('diff_motor_controller')
        self.WHEEL_RADIUS = 0.038
        self.WHEEL_GAP = 0.2
        self.speed = []

        

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
        linear = min(1.0, max(-1.0, data.linear.x))
        angular = min(1.0, max(-1.0, data.angular.z))

        _left_temp = float(((2 * linear) - (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS))
        _right_temp = float(((2 * linear) + (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS))
        print(_left_temp, _right_temp)

        left_temp = ((2 * linear) - (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS)
        right_temp = ((2 * linear) + (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS)
        if data.linear.x != 0:
            left_temp = max(min(1.0, left_temp * 0.01), 0.4) 
            self.motor_driver.motor1.throttle =  left_temp
            self.motor_driver.motor3.throttle = left_temp
            right_temp =  max(min(1.0, right_temp * 0.01), 0.4) 
            self.motor_driver.motor2.throttle = right_temp
            self.motor_driver.motor4.throttle = rright_temp
            rospy.loginfo('FLE: {0}, FRE: {1}, BLE: {2}, BRE:{3}'.format(left_temp, right_temp , round((left_temp / 1000), 2),  round((right_temp / 1000), 2)))
        elif data.angular.z != 0:
            left_temp = max(min(1.0, left_temp * 0.01), 0.4) 
            self.motor_driver.motor1.throttle = left_temp
            self.motor_driver.motor3.throttle = left_temp
            right_temp = max(min(1.0, left_temp * 0.01), 0.4) 
            self.motor_driver.motor2.throttle =  right_temp
            self.motor_driver.motor4.throttle =  right_temp
            rospy.loginfo('FLE: {0}, FRE: {1}, BLE: {2}, BRE:{3}'.format(left_temp, right_temp , round((left_temp / 1000), 2),  round((right_temp / 1000), 2)))
        else:
            self.motor_driver.motor1.throttle = 0
            self.motor_driver.motor2.throttle = 0 
            self.motor_driver.motor3.throttle = 0
            self.motor_driver.motor4.throttle = 0 
        rospy.loginfo('FLE: {0}, FRE: {1}, BLE: {2}, BRE:{3}'.format(left_temp, right_temp , round((left_temp / 1000), 2),  round((right_temp / 1000), 2)))

        print()
    def start_listening(self):
            rospy.Subscriber('/robot/cmd_vel', Twist, self.set_pwm)
            rospy.spin()


if __name__ == '__main__':
    velocity_command = VelocityCommand()
    # _thread.start_new_thread(velocity_command.set_pwm, ())
    velocity_command.start_listening()

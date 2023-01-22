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

        left_temp = ((2 * linear) - (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS)
        right_temp = ((2 * linear) + (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS)
        if data.linear.x != 0:
            left_temp = map((left_temp * 100), -3000, 3000, -1000, 1000)
            self.speed['front_left_expected_speed'] = round((left_temp / 1000), 2)
            self.speed['back_left_expected_speed'] = self.speed['front_left_expected_speed']

            right_temp = map((right_temp * 100), -3000, 3000, -1000, 1000)
            self.speed['front_right_expected_speed'] = round((right_temp / 1000), 2)
            self.speed['back_right_expected_speed'] = self.speed['front_right_expected_speed']
        elif data.angular.z != 0:
            left_temp = map((left_temp * 100), -400, 400, -1000, 1000)
            self.speed['front_left_expected_speed'] = round((left_temp / 1000), 2)
            self.speed['back_left_expected_speed'] = self.speed['front_left_expected_speed']

            right_temp = map((right_temp * 100), -400, 400, -1000, 1000)
            self.speed['front_right_expected_speed'] = round((right_temp / 1000), 2)
            self.speed['back_right_expected_speed'] = self.speed['front_right_expected_speed']
        else:
            # self.speed['front_left_expected_speed'] = 0
            # self.speed['front_right_expected_speed'] = 0
            # self.speed['back_left_expected_speed'] = 0
            # self.speed['back_right_expected_speed'] = 0



        # left_temp = ((2 * linear) - (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS)
        # right_temp = ((2 * linear) + (angular * self.WHEEL_GAP)) / (2 * self.WHEEL_RADIUS)
        # if data.linear.x != 0:
        #     left_temp =  map(float(left_temp * 100), -3000, 3000, -1000, 1000)
        #     self.motor_driver.motor1.throttle =  round((left_temp / 1000), 2)
        #     self.motor_driver.motor3.throttle = round((left_temp / 1000), 2)
        #     right_temp = map(float(right_temp * 100), -3000, 3000, -1000, 1000)
        #     self.motor_driver.motor2.throttle =  round((right_temp / 1000), 2)
        #     self.motor_driver.motor4.throttle = round((right_temp / 1000), 2) 
        #     rospy.loginfo('FLE: {0}, FRE: {1}, BLE: {2}, BRE:{3}'.format(round((left_temp / 1000), 2),  round((right_temp / 1000), 2) , round((left_temp / 1000), 2),  round((right_temp / 1000), 2)))
        # elif data.angular.z != 0:
        #     left_temp = map(float(left_temp * 100), -400, 400, -1000, 1000)
        #     self.motor_driver.motor1.throttle = round((left_temp / 1000), 2)
        #     self.motor_driver.motor3.throttle = round((left_temp / 1000), 2)
        #     right_temp = map(float(right_temp * 100), -400, 400, -1000, 1000)
        #     self.motor_driver.motor2.throttle =  round((right_temp / 1000), 2)
        #     self.motor_driver.motor4.throttle =  round((right_temp / 1000), 2) 
        #     rospy.loginfo('FLE: {0}, FRE: {1}, BLE: {2}, BRE:{3}'.format(round((left_temp / 1000), 2),  round((right_temp / 1000), 2) , round((left_temp / 1000), 2),  round((right_temp / 1000), 2)))
        # else:
            self.motor_driver.motor1.throttle = 0
            self.motor_driver.motor2.throttle = 0 
            self.motor_driver.motor3.throttle = 0
            self.motor_driver.motor4.throttle = 0 
        rospy.loginfo('FLE: {0}, FRE: {1}, BLE: {2}, BRE:{3}'.format(self.speed['front_left_expected_speed'], self.speed['front_right_expected_speed'], self.speed['back_left_expected_speed'], self.speed['back_right_expected_speed']))


        print()
    def start_listening(self):
            rospy.Subscriber('/robot/cmd_vel', Twist, self.set_pwm)
            rospy.spin()


if __name__ == '__main__':
    velocity_command = VelocityCommand()
    # _thread.start_new_thread(velocity_command.set_pwm, ())
    velocity_command.start_listening()

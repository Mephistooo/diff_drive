import rospy
import math
import _thread
import time
from typing import List
from geometry_msgs.msg import Twist
from haruto_msgs.msg import Velocity, PID, PWM
import board
from adafruit_motorkit import MotorKit

class VelocityCommand():
    LINEAR_VELOCITY_MAX_LIMIT = 1.0
    LINEAR_VELOCITY_MIN_LIMIT = -1.0
    ANGULAR_VELOCITY_MAX_LIMIT = 1.0
    ANGULAR_VELOCITY_MIN_LIMIT = -1.0
    WHEEL_RADIUS = 0.038
    WHEEL_GAP = 0.2


    def __init__(self):
        self.min = 0.4
        self.max = 1.0
        self.motor_driver = MotorKit(i2c=board.I2C())
        

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
        linear = min(LINEAR_VELOCITY_MAX_LIMIT, max(LINEAR_VELOCITY_MIN_LIMIT, data.linear.x))
        angular = min(ANGULAR_VELOCITY_MAX_LIMIT, max(ANGULAR_VELOCITY_MIN_LIMIT, data.angular.z))

        left_temp = ((2 * linear) - (angular * WHEEL_GAP)) / (2 * WHEEL_RADIUS)
        right_temp = ((2 * linear) + (angular * WHEEL_GAP)) / (2 * WHEEL_RADIUS)
        if data.linear.x != 0:
            left_temp = round((map((left_temp * 100), -3000, 3000, -1000, 1000) / 1000), 2) 
            self.motor_driver.motor1.throttle = left_temp
            self.motor_driver.motor3.throttle = left_temp
            right_temp =  round((map((right_temp * 100), -3000, 3000, -1000, 1000) / 1000), 2) 
            self.motor_driver.motor2.throttle = right_temp
            self.motor_driver.motor4.throttle = right_temp 
        elif data.angular.z != 0:
            left_temp = round((map((left_temp * 100), -400, 400, -1000, 1000) / 1000), 2) 
            self.motor_driver.motor1.throttle = left_temp
            self.motor_driver.motor3.throttle = left_temp
            right_temp = round((map((right_temp * 100), -400, 400, -1000, 1000) / 1000), 2)
            self.motor_driver.motor2.throttle = right_temp
            self.motor_driver.motor4.throttle = right_temp 
        else:
            self.motor_driver.motor1.throttle = 0
            self.motor_driver.motor2.throttle = 0 
            self.motor_driver.motor3.throttle = 0
            self.motor_driver.motor4.throttle = 0 

    def start_listening(self):
            rospy.Subscriber('/robot/cmd_vel', Twist, self.set_pwm)
            rospy.spin()


if __name__ == '__main__':
    velocity_command = VelocityCommand()
    # _thread.start_new_thread(velocity_command.set_pwm, ())
    velocity_command.start_listening()

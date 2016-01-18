#!/usr/bin/env python
import rospy
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import LaserScan # for the laser data
from racecar.msg import wd

import matplotlib.pyplot as plt

# define the method for generating a one dimensional MultiArray
def makeMultiArray(iterable, label):
    arrayList = []
    for el in iterable:
        arrayList.append(el)
    dim = MultiArrayDimension()
    dim.size = len(arrayList)
    dim.label = label
    dim.stride = len(arrayList)
 
    tempArray = Float64MultiArray()
    tempArray.data = arrayList
    tempArray.layout.dim.append(dim)
    tempArray.layout.data_offset = 0
    return tempArray

class _AckermannCtrlr(object):
    """Ackermann controller
    An object of class _AckermannCtrlr is a node that controls the wheels of a
    vehicle with Ackermann steering.
    """

    def __init__(self):
        plt.ion()

        # initializing node
        rospy.init_node("acker_ctrl") # jetson_ackermann_controller

        self._sleep_timer = rospy.Rate(10)

        self._steer_ang = 0.0       # Steering angle
        self._last_steer_ang = 0.0  # Last steering angle
        self._steer_ang_vel = 0.0   # Steering angle velocity

        # Acceleration
        self._speed = 0.0
        self._accel = 0.0
        self._jerk = 0.0

        # Walls definition
        self._right_wall_a = 0
        self._right_wall_b = 0
        self._left_wall_a = 0
        self._left_wall_b = 0
        self._middle_line_a = 0
        self._middle_line_b = 0
        self._prev_middle_line_b = 0

        # Object with the message to be published
        self._ackermann_cmd = AckermannDriveStamped()
        # header
        self._ackermann_cmd.header.seq = 0
        self._ackermann_cmd.header.stamp.secs = 0
        self._ackermann_cmd.header.stamp.nsecs = 0
        self._ackermann_cmd.header.frame_id = ''
        # driver
        self._ackermann_cmd.drive.speed = 0
        self._ackermann_cmd.drive.acceleration = 0
        self._ackermann_cmd.drive.jerk = 0
        self._ackermann_cmd.drive.steering_angle = 0
        self._ackermann_cmd.drive.steering_angle_velocity = 0

        # Publishers
        self._ackermann_cmd_pub = rospy.Publisher("/vesc/ackermann_cmd", AckermannDriveStamped, queue_size=10)

        # Subscribers
        # laser data
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)
        # wall data
        rospy.Subscriber("/wd", wd, self.wall_callback, queue_size=10)

        self.sign = 1

    def laser_callback(self, laser_data):
        # update laser data
        return
        

    def wall_callback(self, walls_data):
        # update wall data
        #left wall
        self._left_wall_a = walls_data.a_l
        self._left_wall_b = walls_data.b_l
        #right wall
        self._right_wall_a = walls_data.a_r
        self._right_wall_b = walls_data.b_r

        # finding the coefficientes of the middle line
        # y = h x + k
        alpha = math.atan(self._left_wall_a)
        beta = math.atan(self._right_wall_a)
        h = math.tan((alpha + beta)/2.0)
        k = (self._right_wall_b - self._left_wall_b)*(self._left_wall_a - h)/(self._left_wall_a - self._right_wall_a) + self._left_wall_b
        
        self._middle_line_a = h
        self._prev_middle_line_b = self._middle_line_b
        self._middle_line_b = k

        # zr = []
        # zr.append(walls_data.a_r)
        # zr.append(walls_data.b_r)
        # zl = []
        # zl.append(walls_data.a_l)
        # zl.append(walls_data.b_l)
        # zm = []
        # zm.append(self._middle_line_a)
        # zm.append(self._middle_line_b)

        # pr = np.poly1d(zr)
        # xpr = np.linspace(-15, 15, 100)
        # pl = np.poly1d(zl)
        # xpl = np.linspace(-15, 15, 100)
        # pm = np.poly1d(zm)
        # xpm = np.linspace(-15, 15, 100)

        # plt.clf()
        # plt.plot(xpr, pr(xpr), 'r-', xpl, pl(xpl), 'g-', xpm, pm(xpm), 'b-')
        # plt.axis([-15,15,-15,15])
        # plt.draw()
        
        pass
        
    def spin(self):
        """Control the vehicle."""
        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t

            # header
            self._ackermann_cmd.header.seq = self._ackermann_cmd.header.seq + 1
            

            # Velocity Controller
            self._ackermann_cmd.drive.speed = self._ctrl_speed() # range = -2:2
            self._ackermann_cmd.drive.acceleration = 0 # range = 0:0, always 0 from the joystick
            self._ackermann_cmd.drive.jerk = 0 # range = 0:0, always 0 from the joystick

            # Steering Controller
            self._ackermann_cmd.drive.steering_angle = self._ctrl_steering() # range = -0.34:0.34
            self._ackermann_cmd.drive.steering_angle_velocity = 0 # range = 0:0 , always 0 from the joystick

            # Publish the velocity
            self._ackermann_cmd_pub.publish(self._ackermann_cmd)

            # wait
            self._sleep_timer.sleep()

    def _ctrl_speed(self):
        _ctrl_speed_input = 0.5

        return _ctrl_speed_input

    def _ctrl_steering(self):
        # Just witching the steering every second for testing purposes
        #-----------------------------------------------------------------------
        # if (self._ackermann_cmd.header.seq % 10 == 0):
        #         self.sign = -self.sign
        # _ctrl_steering_input = 0.34*self.sign
        

        # Distance to the wall approach
        #-----------------------------------------------------------------------
        # we want to minimize the distance between the robot and the middle line
        # so the error is the linear coefficient of the middle line (_middle_line_b)
        kP = 0.34 # proportional constant
        kD = 0.1 # derivative constant

        # Proportional Control
        _ctrl_steering_input = kP*self._middle_line_b

        # Proportional + Derivative
        # _ctrl_steering_input = kD*(self._prev_middle_line_b - self._middle_line_b)


        return _ctrl_steering_input


 # main
if __name__ == "__main__":
    ctrlr = _AckermannCtrlr()
    ctrlr.spin()
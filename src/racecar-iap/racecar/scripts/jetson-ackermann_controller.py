#!/usr/bin/env python
import rospy
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool
from sensor_msgs.msg import LaserScan, Joy # for the laser data
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

        self._sleep_timer = rospy.Rate(5)

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
        self._ki_val_pub = rospy.Publisher("/vesc/ki_val", Float64, queue_size=10)

        # Subscribers
        # laser data
        # rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)
        # wall data
        rospy.Subscriber("/vesc/wd", wd, self.wall_callback, queue_size=20)
        # danger data
        rospy.Subscriber("/vesc/danger", Float64, self.danger_callback, queue_size=20)
        # joy data
        rospy.Subscriber("/vesc/joy", Joy, self.joy_callback, queue_size=20)

        self.sign = 1
        self.danger = 0
        self.joy_command = [0]*12
        self._int_error_sum = 0
        self._dist_right_wall = 0
        self._dist_left_wall = 0
        self._has_corner = False
        self._corner_x = 0
        self._corner_y = 0
        self._xo = 1
        self._yo = 0
        self.dist_total = 0
        self.parallel = 0

        self.ki_button_pressed = False
        self.ki_button_pressed_time = rospy.get_time()
        self._ki = 1

    # def laser_callback(self, laser_data):
    #     # update laser data
    #     return
        

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

        self._dist_left_wall = self._left_wall_b/self._left_wall_a * math.sin(math.atan(self._left_wall_a))
        self._dist_right_wall = -self._right_wall_b/self._right_wall_a * math.sin(math.atan(self._right_wall_a))
        self._has_corner = walls_data.corner_bool
        self._corner_x = walls_data.corner_x
        self._corner_y = walls_data.corner_y

        shift = 1.5
        if self._has_corner:
            # print("Corner steering_angle: " + str(math.atan(self._corner_y/self._corner_x)/math.pi*180))
            # print("Corner range: " + str(math.sqrt(self._corner_x**2 + self._corner_y**2)))      
            self._xo = self._corner_x + shift*math.cos(math.atan(self._right_wall_a))
            self._yo = self._corner_y + shift*math.sin(math.atan(self._right_wall_a))
        else:
            alpha = math.atan(self._left_wall_a)
            beta = math.atan(self._right_wall_a)
            a = math.tan((alpha + beta)/2.0)
            b = (self._right_wall_b - self._left_wall_b)*(self._left_wall_a - a)/(self._left_wall_a - self._right_wall_a) + self._left_wall_b
            self._xo = 5
            self._yo = a*5 + b - 1 # trying to correct left bias

        # Distance between the walls, considering the robot position
        dist_right = abs(self._right_wall_b)/math.sqrt(self._right_wall_a**2 + 1)
        dist_left = abs(self._left_wall_b)/math.sqrt(self._left_wall_a**2 + 1)
        self.dist_total = dist_right + dist_left
        self.parallel = abs(self._right_wall_a - self._left_wall_a) < math.tan(10/180*math.pi)


        # print("Middle Angular Coefficient: " + str(self._middle_line_a))
        # print("Right Angular Coefficient: " + str(self._right_wall_a))
        # print("Left Angular Coefficient: " + str(self._left_wall_a))

        # Ploting Results
        # zr = []
        # zr.append(walls_data.a_r)
        # zr.append(walls_data.b_r)
        # zl = []
        # zl.append(walls_data.a_l)
        # zl.append(walls_data.b_l)
        # # zm = []
        # # zm.append(self._middle_line_a)
        # # zm.append(self._middle_line_b)

        # pr = np.poly1d(zr)
        # xpr = np.linspace(-15, 15, 100)
        # pl = np.poly1d(zl)
        # xpl = np.linspace(-15, 15, 100)
        # # pm = np.poly1d(zm)
        # # xpm = np.linspace(-15, 15, 100)

        # plt.clf()
        # plt.plot(xpr, pr(xpr), 'r-', xpl, pl(xpl), 'b-')
        # plt.axis([-15,15,-15,15])
        # plt.plot(self._xo, self._yo, 'gx')
        # if self._has_corner:
        #     plt.plot(self._corner_x, self._corner_y, 'g^')
        # plt.draw()
        
        pass

    def danger_callback(self, danger_data):
        # update danger data
        self.danger = danger_data.data

    def joy_callback(self, joy_data):
        # update danger data
        self.joy_command = joy_data.buttons
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
            self._ackermann_cmd.drive.speed = self._ctrl_speed()
            self._ackermann_cmd.drive.acceleration = 0
            self._ackermann_cmd.drive.jerk = 0

            # Steering Controller
            self._ackermann_cmd.drive.steering_angle = self._ctrl_steering()
            self._ackermann_cmd.drive.steering_angle_velocity = 0

            # Publish the velocity
            if self.joy_command[4] != 1:
                self._ackermann_cmd_pub.publish(self._ackermann_cmd)

            # Publish ki
            if self.ki_button_pressed and (t-self.ki_button_pressed_time > .15):
                self.ki_button_pressed = False

            if not self.ki_button_pressed:
                if (self.joy_command[3] == 1):
                    self._ki = self._ki*2
                    ki_msg = Float64()
                    ki_msg.data = self._ki
                    self._ki_val_pub.publish(ki_msg)
                    self.ki_button_pressed = True
                    self.ki_button_pressed_time = t
                elif (self.joy_command[1] == 1):
                    self._ki = self._ki/2
                    ki_msg = Float64()
                    ki_msg.data = self._ki
                    self._ki_val_pub.publish(ki_msg)
                    self.ki_button_pressed = True
                    self.ki_button_pressed_time = t


            # wait
            self._sleep_timer.sleep()

    def _ctrl_speed(self):
        if self.joy_command[4] == 1:
            _ctrl_speed_input = 0
        else:
            # if there is a corner, go slower
            if self._has_corner:
                if self.danger == 0:
                    _ctrl_speed_input = 4
                elif self.danger == 1:
                    _ctrl_speed_input = 0.5
                elif self.danger == 2:
                    _ctrl_speed_input = 1
                elif self.danger == 3:
                    _ctrl_speed_input = 3

                # print("Speed: " + str(_ctrl_speed_input*3000))
                # _ctrl_speed_input = 0


                #_ctrl_speed_input = 6 if self.danger == 0 else 3000*self.danger
            else:
                #_ctrl_speed_input = 6 if self.danger == 0 else 3000*self.danger
                if self.danger == 0:
                    _ctrl_speed_input = 6 if self.dist_total > 3.5 else 4
                elif self.danger == 1:
                    _ctrl_speed_input = 0.5
                elif self.danger == 2:
                    _ctrl_speed_input = 1
                elif self.danger == 3:
                    _ctrl_speed_input = 3

        # print("Parallel: " + str(self.parallel))
        # if self.dist_total and self._has_corner > 3.5:
        #     print("Drive fast!""Drive fast!""Drive fast!""Drive fast!")
        # print("Angular difference: " + str(abs(self._right_wall_a - self._left_wall_a)))
        # print("Distance bet walls: " + str(self.dist_total))
        # print("Speed: " + str(_ctrl_speed_input*3000))
                # _ctrl_speed_input = 0
        return _ctrl_speed_input

    def _ctrl_steering(self):
        #kP = 0.5 # for low velocity
        kP = 0.2 # for high velocity
        kD = 0.1 
        kI = 0 # self._ki

        self._int_error_sum = self._int_error_sum + math.atan2(self._yo, self._xo)

        saturation = 0.05
        if kI * self._int_error_sum > saturation:
            self._int_error_sum = saturation/kI
        elif kI * self._int_error_sum < -saturation:
            self._int_error_sum = -saturation/kI


        _ctrl_steering_input = kP*math.atan2(self._yo, self._xo) + kI*self._int_error_sum

        # print("steering_angle: " + str(_ctrl_steering_input)) 
        if _ctrl_steering_input > 0.34:
            _ctrl_steering_input = 0.34
        elif _ctrl_steering_input < -0.34:  
            _ctrl_steering_input = -0.34

        # print(" ")
        return _ctrl_steering_input


 # main
if __name__ == "__main__":
    ctrlr = _AckermannCtrlr()
    ctrlr.spin()

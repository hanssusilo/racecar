#!/usr/bin/python
# import main ROS python library
import rospy
import ransac
import numpy as np
import matplotlib.pyplot as plt
import math

# import the Laser Scan message type
from sensor_msgs.msg import LaserScan
from racecar.msg import wd
# simple class to contain the node's variables and code

class WallDetector:
	# class constructor; subscribe to topics and advertise intent to publish
	def __init__(self):
		# subscribe to the laser scan topic
		print("Wall detector started")
		self.wd_subs = rospy.Subscriber("/scan", LaserScan, self.wall_detector_callback)
		self.wd_pub = rospy.Publisher("wd", wd, queue_size=10)
		self.corner_thresh = 4
		plt.ion()

		self.window=10.0
		self.confidence_num=5
		self.watch_window_min=-np.pi/6
		self.watch_window_max=-np.pi*2/3.0

	def distance_to_the_wall(self, angle, wall_data):
		h_distance=abs(wall_data)
		return h_distance/np.cos(angle)

	# the callback function for the number stream topic subscription
	def wall_detector_callback(self, msg):
		# Right wall points
		right_wall = msg.ranges[240:480]
		angle = msg.angle_min + msg.angle_increment*240
		xr = []
		yr = []
		for dist in right_wall:
			if dist < msg.range_max:
				xr.append(dist*np.cos(angle))
				yr.append(dist*np.sin(angle))
			angle = angle + msg.angle_increment

		# Left wall points
		left_wall = msg.ranges[600:840]
		angle = msg.angle_min + msg.angle_increment*600
		xl = []
		yl = []
		for dist in left_wall:
			if dist < msg.range_max:
				xl.append(dist*np.cos(angle))
				yl.append(dist*np.sin(angle))
			angle = angle + msg.angle_increment

		# Finding wall equation
		zr,residuals_r, rank_r, singular_values_r, rcond_r = np.polyfit(xr, yr, 1, full=True)
		zl,residuals_l, rank_l, singular_values_l, rcond_l = np.polyfit(xl, yl, 1, full=True)
		residuals_l = residuals_l/len(xl)
		residuals_r = residuals_r/len(xr)
		
		# Corner detection
		corner = msg.ranges[60:480]
		i = 60
		index = []
		for dist in corner:
			if dist < msg.range_max:
				index.append(i)
			i += 1

		corner = [msg.ranges[i] for i in index]
		i = 0
		corner_bool = False
		corner_x = 0
		corner_y = 0
		while i < len(corner)-5:
			if (corner[i+1] - corner[i] > 1.5):
				corner_x = corner[i]*np.cos(msg.angle_min + msg.angle_increment*index[i])
				corner_y = corner[i]*np.sin(msg.angle_min + msg.angle_increment*index[i])
				# print((msg.angle_min + msg.angle_increment*index[i])*180/np.pi)
				corner_bool = True
				break
			i += 1

		# Publishing message
		message = wd()
		message.a_r = zr[0]
		message.b_r = zr[1]
		message.a_l = zl[0]
		message.b_l = zl[1]
		message.corner_bool = corner_bool
		message.corner_x = corner_x
		message.corner_y = corner_y

		self.wd_pub.publish(message)
		
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
		
		if corner_bool:
			# print("Corner angle: " + str(math.atan(corner_y/corner_x)/math.pi*180))
			# print("Corner range: " + str(math.sqrt(corner_x**2 + corner_y**2)))      
			xo = corner_x + 1.5*math.cos(math.atan(message.a_r))
			yo = corner_y + 1.5*math.sin(math.atan(message.a_r))
		else:
			alpha = math.atan(message.a_l)
			beta = math.atan(message.a_r)
			a = math.tan((alpha + beta)/2.0)
			b = (message.b_r - message.b_l)*(message.a_l - a)/(message.a_l - message.a_r) + message.b_l
			xo = 5
			yo = a*5 + b - 1

		# Plotting results
		angle = msg.angle_min
		x = []
		y = []
		for dist in msg.ranges:
			if dist < msg.range_max:
				x.append(dist*np.cos(angle))
				y.append(dist*np.sin(angle))
		 	angle = angle + msg.angle_increment

		pr = np.poly1d(zr)
		xpr = np.linspace(-15, 15, 100)
		pl = np.poly1d(zl)
		xpl = np.linspace(-15, 15, 100)

		plt.clf()
		# plt.plot(x,y,'.', xpr, pr(xpr), '-', xpl, pl(xpl), '-', xr, yr, 'ro', xl, yl, 'ro')
		plt.plot(x,y,'.', xpr, pr(xpr), '-', xpl, pl(xpl), '-', xr, yr, 'ro', xl, yl, 'ro')
		plt.plot(0,0, 'o', markersize=10)
		if corner_bool:
			plt.plot(corner_x, corner_y, 'rx', markersize=5)
		plt.plot(xo, yo, 'gx', markersize=5)
		
		plt.axis([-15,15,-15,15])
		plt.draw()
		plt.show()

if __name__ == "__main__":
	# initialize the ROS client API, giving the default node name
	rospy.init_node("WD_node")
	node = WallDetector()
	# enter the ROS main loop
	rospy.spin()

#!/usr/bin/python
# import main ROS python library
import rospy
import ransac
import numpy as np
import matplotlib.pyplot as plt

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
		self.wd_pub = rospy.Publisher("wd", wd)
		plt.ion()

	# the callback function for the number stream topic subscription
	def wall_detector_callback(self, msg):
		# Right wall points
		right_wall = msg.ranges[180:480]
		angle = msg.angle_min + msg.angle_increment*180
		xr = []
		yr = []
		for dist in right_wall:
			if dist < msg.range_max:
				xr.append(dist*np.cos(angle))
				yr.append(dist*np.sin(angle))
			angle = angle + msg.angle_increment

		# Left wall points
		left_wall = msg.ranges[600:900]
		angle = msg.angle_min + msg.angle_increment*600
		xl = []
		yl = []
		for dist in left_wall:
			if dist < msg.range_max:
				xl.append(dist*np.cos(angle))
				yl.append(dist*np.sin(angle))
			angle = angle + msg.angle_increment

		# Finding wall equation
		zr = np.polyfit(xr, yr, 1)
		zl = np.polyfit(xl, yl, 1)
		
		# Publishing message
		message = wd()
		message.a_r = zr[0]
		message.b_r = zr[1]
		message.a_l = zl[0]
		message.b_l = zl[1]

		self.wd_pub.publish(message)

		# Plotting results
'''		angle = msg.angle_min
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
		plt.plot(x,y,'.', xpr, pr(xpr), '-', xpl, pl(xpl), '-', xr, yr, 'ro', xl, yl, 'ro')
		plt.axis([-15,15,-15,15])
		plt.draw()
'''		

		# n_inputs = 1
		# n_outputs = 1
		# debug = False
		# X = np.asarray([x])
		# Y = np.asarray([y])
		# X = np.transpose(X)
		# Y = np.transpose(Y)
		# all_data = np.hstack( (X,Y) )
		# input_columns = range(n_inputs) # the first columns of the array
		# output_columns = [n_inputs+i for i in range(n_outputs)] # the last columns of the array
		# model = ransac.LinearLeastSquaresModel(input_columns,output_columns,debug=debug)
		# # run RANSAC algorithm
		# ransac_fit, ransac_data = ransac.ransac(all_data,model, 30, 100, 7e3, 60, debug=debug,return_all=True)
		# print(ransac_fit)
		#wd_subs.unregister()


				



if __name__ == "__main__":
	# initialize the ROS client API, giving the default node name
	rospy.init_node("WD_node")
	node = WallDetector()
	# enter the ROS main loop
	rospy.spin()

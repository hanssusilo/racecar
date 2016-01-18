#!/usr/bin/python
import rospy
import ransac
import numpy as np
import matplotlib.pyplot as plt

# import the Laser Scan message type
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class DangerDetector:

	def __init__(self):
		# subscribe to the laser scan topic
		print("DangerDedector started")

		# Initialiazing the node
		rospy.init_node("DangerDedector")

		# Publishers
		self._danger_pub = rospy.Publisher("danger", Bool, queue_size = 10)

		# Subscribers
		rospy.Subscriber("/scan", LaserScan, self.readingLaser)

		# Ploting
		plt.ion()

	def readingLaser(self, msg):
		''' readingLaser(self, msg):
		Verifies obstacles ahead of the laser scan.
		'''
		'''observations about the laser scanner:
		- range stands for the distance measured by the laser
		- The data goes from -135 to 135 degrees with increments of 0.25
		- Angle -90: 180, Angle 90: 600, Angle 0: 540
		'''

		range_min = 0.2 # MARK: verify what is ther actual range of values
		obs_free_limit = 36

		# Front points
		#init = 480
		#end = 600
		# defining a specific range of angles to be considered
		front_obs_ranges = msg.ranges[480:600]
		weights = [0 if dist > range_min else 1 for dist in front_obs_ranges]
		danger_ahead = sum(weights) > obs_free_limit

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
		plt.plot(x,y,'.', xpr, pr(xpr), '-', xpl, pl(xpl), '-', xr, yr, 'ro', xl, yl, 'ro')
		if danger_ahead:
			plt.plot(0, 0, 'g^')

		plt.axis([-15,15,-15,15])
		plt.draw()
		plt.show()

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
	node = DangerDetector()
	rospy.spin()

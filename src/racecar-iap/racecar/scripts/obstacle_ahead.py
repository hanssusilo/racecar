#!/usr/bin/python
import rospy
import ransac
import numpy as np
import matplotlib.pyplot as plt

# import the Laser Scan message type
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class DangerDetector:

	def __init__(self):
		# subscribe to the laser scan topic
		print("DangerDedector started")

		# Initialiazing the node
		rospy.init_node("DangerDedector")

		# Publishers
		self._danger_pub = rospy.Publisher("danger", Float64, queue_size = 10)

		# Subscribers
		rospy.Subscriber("/scan", LaserScan, self.readingLaser)
		rospy.Subscriber("/vesc/ackermann_cmd", AckermannDriveStamped, self._steering_data)

		self._steering_angle = 0

		# Ploting
		plt.ion()

	def _steering_data(self, ackermann_cmd):
		self._steering_angle = ackermann_cmd.drive.steering_angle
		pass

	def readingLaser(self, msg):
		''' readingLaser(self, msg):
		Verifies obstacles ahead of the laser scan.
		'''
		'''observations about the laser scanner:
		- range stands for the distance measured by the laser
		- The data goes from -135 to 135 degrees with increments of 0.25
		- Angle -90: 180, Angle 90: 600, Angle 0: 540
		'''
		# looking to the steering angle direction
		# 0.34 -> 1080
		# -0.34 -> 0
		range_middle = int((self._steering_angle + 0.34)/0.68*1080)
		init = range_middle - 120
		end = range_middle + 120
		# init = 420
		# end = 660
		init = 480
		end = 600

		obs_free_limit = 50

		# For just looking straight ahead:
		# Front points # 480, 600 for 30 degrees # 420, 660 for 60 degrees
		# init = 420
		# end = 660
		# defining a specific range of angles to be considered

		range_1 = 1 # MARK: verify what is ther actual range of values
		range_2 = 2
		range_3 = 3 
		# range_2 = 1.67
		# range_3 = 2.33
		# range_4 = 3

		front_obs_ranges = msg.ranges[init:end]
		weights_1 = [0 if dist > range_1 else 1 for dist in front_obs_ranges]
		weights_2 = [0 if dist > range_2 or dist < range_1 else 1 for dist in front_obs_ranges]
		weights_3 = [0 if dist > range_3 or dist < range_2 else 1 for dist in front_obs_ranges]
		sum_weights = [sum(weights_1), sum(weights_2), sum(weights_3)]
		max_indice = sum_weights.index(max(sum_weights))
		has_obstacle = sum_weights[max_indice] > obs_free_limit
		if has_obstacle:
			danger_ahead = max_indice + 1
		else:
			danger_ahead = 0

		print("Zone: " + str(max_indice))
		# weights_1 = [0 if dist > range_min else 1 for dist in front_obs_ranges]
		# weights_2 = [0 if dist > range_min else 1 for dist in front_obs_ranges]
		# weights_3 = [0 if dist > range_min else 1 for dist in front_obs_ranges]
		# weights_4 = [0 if dist > range_min else 1 for dist in front_obs_ranges]

		self._danger_pub.publish(danger_ahead)
		'''
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
	node = DangerDetector()
	rospy.spin()

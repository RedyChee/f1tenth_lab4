#!/usr/bin/env python3

import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
	def __init__(self):
		self.lidar_sub = rospy.Subscriber('/scan' , LaserScan, self.lidar_callback, queue_size = 1)
		self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1)

	def preprocess_lidar(self, ranges):
		""" Preprocess the LiDAR scan array. Expert implementation includes:
				1.Setting each value to the mean over some window
				2.Rejecting high values (eg. > 3m)
		"""
		proc_ranges = ranges
		return proc_ranges

	def find_max_gap(self, free_space_ranges):
		""" Return the start index & end index of the max gap in free_space_ranges
		"""
		return None

	def find_best_point(self, start_i, end_i, ranges):
		"""Start_i & end_i are start and end indicies of max-gap range, respectively
		Return index of best point in ranges
		Naive: Choose the furthest point within ranges and go there
		"""
		return None

	def lidar_callback(self, data):
		""" Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
		"""
		ranges = data.ranges
		proc_ranges = self.preprocess_lidar(ranges)

		#Find closest point to LiDAR

		#Eliminate all points inside 'bubble' (set them to zero) 

		#Find max length gap 

		#Find the best point in the gap 

		#Publish Drive message

def main(args):
	rospy.init_node("FollowGap_node", anonymous=True)
	rfgs = reactive_follow_gap()
#	rospy.sleep(0.1)
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

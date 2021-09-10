#!/usr/bin/env python3
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

CAR_WIDTH = 0.2032

class reactive_follow_gap:
	def __init__(self):
		self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size = 1)
		self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1)

	def preprocess_lidar(self, scan_msg):
		""" Preprocess the LiDAR scan array. Expert implementation includes:
				1.Setting each value to the mean over some window
				2.Rejecting high values (eg. > 3m)
		"""
		# 140 degrees front angle [110, 250]
		scan_range = np.asarray(scan_msg.ranges)
#		print(scan_range.size)
		ranges = scan_range[int(110/360*scan_range.size)-2:int(250/360*scan_range.size)+3]
		window = 5
		deno = window*(window+1)/2
		weight = np.array([1, 2, 3, 4, 5])/deno
		idx = 2
		rolling_mean = np.array([])
		# rolling average
		while idx <= (ranges.size-3):
			dist = ranges[idx-2:idx+3]
			mean = np.matmul(dist, weight)
			rolling_mean = np.append(rolling_mean, mean)
			idx += 1
		# accept distance <= 3
		filter_rmean = np.array([])
		for val in rolling_mean:
			if val > 3:
				val = 3
			filter_rmean = np.append(filter_rmean, val)
		proc_ranges = filter_rmean
		return proc_ranges

#	def disparity(self, start, end, data):
#		angle_increment = data.angle_increment
#		scan_range = np.asarray(data.ranges)
#		self.ranges = scan_range[int(120/360*scan_range.size):int(240/360*scan_range.size)+1]
#		disparity = 0.05
#		obs = self.ranges
#		turn = None # right=0, left=1
#		for idx in range(obs.size):
#			if idx == obs.size-1:
#				break
#			else:
#				diff = abs(obs[idx] - obs[idx+1])
#				if diff > disparity:
#					disparity = diff
#					start_idx = idx
#		if obs[start_idx] < obs[start_idx+1]:
#			oh = (CAR_WIDTH+0.1)/obs[start_idx]
#			turn = 0
#		elif obs[start_idx+1] < obs[start_idx]:
#			oh = (CAR_WIDTH+0.1)/obs[start_idx+1]
#			turn = 1
#		extend = np.arcsin(oh)
#		print(oh)
#		print(extend)
#		print(extend//angle_increment)
#		num = int(extend//angle_increment)
#		if turn == 0:
#			best_point = start_idx+num
#		elif turn == 1:
#			best_point = start_idx+1-num
#		print(disparity)
#		print(start_idx)
#		print(obs[start_idx])
#		print(obs[start_idx+1])
#		print(extend*180/np.pi)
#		print(num)
#		print(best_point)
#		return best_point

	def find_free_space(self, proc_ranges):
		#Find closest point to LiDAR
		#Eliminate all points inside 'bubble' (set them to zero)
		min_range = min(proc_ranges)	
		min_idx = np.argmin(proc_ranges)
#		print(min_range)
#		print(min_idx)
		min_angle = ((min_idx/(proc_ranges.size-1))*140 + 110 - 90)*(np.pi/180)
#		print("min_angle = ", min_angle*180/np.pi)
		min_x = min_range*np.cos(min_angle)
		min_y = min_range*np.sin(min_angle)
		radius = 5
		idx = 0
		for dist in proc_ranges:
			angle = ((idx/(proc_ranges.size-1))*140 + 110 - 90)*(np.pi/180)
			x = dist*np.cos(angle)
			y = dist*np.sin(angle)
			d_square = (x-min_x)**2 + (y-min_y)**2
			if d_square < radius:
#				print(d_square)
				proc_ranges[idx] = 0
			idx += 1
#		print(proc_ranges)
#		print()
		return proc_ranges

	def find_max_gap(self, free_space_ranges):
		""" Return the start index & end index of the max gap in free_space_ranges
		"""
		start_idx = None
		max_gap = 0
		for idx in range(free_space_ranges.size):
			if free_space_ranges[idx] != 0:
				if start_idx is None:
					start_idx = idx
#					print(start_idx)
					if idx+1 == free_space_ranges.size:
						max_gap = 1
						max_start_idx = start_idx
						max_end_idx = None
						break
					else:
						if free_space_ranges[idx+1] == 0:
							gap = 1
							if gap >= max_gap:
								max_gap = gap
								max_start_idx = start_idx
								max_end_idx = None
							start_idx = None
				else:
					if idx+1 == free_space_ranges.size:
						end_idx = idx
						gap = free_space_ranges[start_idx:end_idx+1].size
#						print(end_idx)
#						print(gap)
						if gap >= max_gap:
							max_gap = gap
							max_start_idx = start_idx
							max_end_idx = end_idx
						break
					else:
						if free_space_ranges[idx+1] == 0:
							end_idx = idx
							gap = free_space_ranges[start_idx:end_idx+1].size
#							print(end_idx)
#							print(gap)
							if gap >= max_gap:
								max_gap = gap
								max_start_idx = start_idx
								max_end_idx = end_idx
							start_idx = None
#		print()
#		print(max_start_idx)
#		print(max_end_idx)
#		print(free_space_ranges)
		return max_start_idx, max_end_idx

	def find_best_point(self, start_i, end_i, ranges):
		"""Start_i & end_i are start and end indicies of max-gap range, respectively
		Return index of best point in ranges
		Naive: Choose the furthest point within ranges and go there
		"""
		max_gap_range = ranges[start_i:end_i+1]
		furthest_point = max(max_gap_range)
		start_idx = None
		max_free = 0
		if (start_i is not None) and (end_i is None):
			best_point = start_i
		if (start_i is not None) and (end_i is not None):
			for idx in range(max_gap_range.size):
				if max_gap_range[idx] == furthest_point:
					if start_idx is None:
						start_idx = idx
#						print(start_idx)
						if idx+1 == max_gap_range.size:
							max_free = 1
							max_start_idx = start_idx
							max_end_idx = None
							break
						else:
							if max_gap_range[idx+1] != furthest_point:
								free = 1
								if free >= max_free:
									max_free = free
									max_start_idx = start_idx
									max_end_idx = None
								start_idx = None
					else:
						if idx+1 == max_gap_range.size:
							end_idx = idx
							free = max_gap_range[start_idx:end_idx+1].size
#							print(end_idx)
#							print(free)
							if free >= max_free:
								max_free = free
								max_start_idx = start_idx
								max_end_idx = end_idx
							break
						else:
							if max_gap_range[idx+1] != furthest_point:
								end_idx = idx
								free = max_gap_range[start_idx:end_idx+1].size
#								print(end_idx)
#								print(free)
								if free >= max_free:
									max_free = free
									max_start_idx = start_idx
									max_end_idx = end_idx
								start_idx = None
				else:
					if idx == max_gap_range.size:
						start 
			if (max_start_idx is not None) and (max_end_idx is None):
				best_point = max_start_idx + start_i
			if (max_start_idx is not None) and (max_end_idx is not None):
				centerline = len(ranges)/2
#				print("center= ", centerline)
				diff_arr = np.array([])
#				for n in range(max_start_idx, max_end_idx+1):
#					diff = abs(centerline - (n + start_i))
#					diff_arr = np.append(diff_arr, diff)
#				best_point = np.argmin(diff_arr) + max_start_idx + start_i
				center_max_gap = (max_start_idx + max_end_idx)/2 + start_i
				if center_max_gap <= (4.5/10)*len(ranges):
					print("part 1")
					best_point = 4*(max_start_idx + max_end_idx)/5 + start_i
				elif (4.5/10)*len(ranges) < center_max_gap < (5.5/10)*len(ranges):
					print("part 2")
					for n in range(max_start_idx, max_end_idx+1):
						diff = abs(centerline - (n + start_i))
						diff_arr = np.append(diff_arr, diff)
					best_point = np.argmin(diff_arr) + max_start_idx + start_i
				elif center_max_gap >= (5.5/10)*len(ranges):
					print("part 3")
					best_point = 1*(max_start_idx + max_end_idx)/5 + start_i
#		print(furthest_point)
#		print(max_start_idx)
#		print(max_end_idx)
#		print(ranges)
#		print(ranges.size)
#		print(best_point)
		return best_point

	def lidar_callback(self, data):
		""" Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
		"""
		proc_ranges = self.preprocess_lidar(data)
		free_ranges = self.find_free_space(proc_ranges)
		# Find max length gap 
		start, end = self.find_max_gap(free_ranges)
		# Find the best point in the gap
#		best_point = self.disparity(start, end, data)
		best_point = self.find_best_point(start, end, free_ranges)
		# Publish Drive message
#		angle = ((best_point/(self.ranges.size-1))*70 - 35)*(np.pi/180)
		angle = ((best_point/(free_ranges.size-1))*140 - 70)*(np.pi/180)
#		print("angle = ", angle*180/np.pi)
#		print()
		if -np.pi/18 < angle < np.pi/18:
			velocity = 1.5
		elif -np.pi/9 < angle <= -np.pi/18 or np.pi/18 <= angle < np.pi/9:
			velocity = 1
		else:
			velocity = 0.5
		drive_msg = AckermannDriveStamped()
		drive_msg.header.stamp = rospy.Time.now()
		drive_msg.header.frame_id = "laser"
		drive_msg.drive.steering_angle = angle
		drive_msg.drive.speed = velocity
		self.drive_pub.publish(drive_msg)		

def main(args):
	rospy.init_node("chee0134_follow_gap", anonymous=True)
	rfgs = reactive_follow_gap()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

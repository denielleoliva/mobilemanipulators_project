#!/usr/bin/python3

import copy

import rospy

from geometry_msgs.msg import WrenchStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan

class OccupancyGridMap2D:

	def __init__(self):
		self.grid = []
		self.cost_grid = []

		self.width = 200
		self.height = 200
		self.resolution = .05

		for i in range(self.width):
			row = []
			for j in range(self.height):
				row.append(0)

			self.grid.append(row)
			self.cost_grid.append(copy.deepcopy(row))

	def reset_map(self):
		for row in self.grid:
			for col in range(len(row)):
				row[col] = 0

	def reset_costs(self):
		for row in self.cost_grid:
			for col in range(len(row)):
				row[col] = 0

	def init_costs(self, robot_positions):
		for row in range(len(self.cost_grid)):
			for col in range(len(self.cost_grid[row])):
				if [row, col] in robot_positions:
					self.cost_grid[row][col] = 0
				else:
					self.cost_grid[row][col] = 1000000

	def init_utils(self):
		for row in range(len(self.cost_grid)):
			for col in range(len(self.cost_grid[row])):
				self.util_grid[row][col] = 0

	def init_from_scan(self, point_cloud):
		self.resetMap()

		# then yaknow init from velodyne lol

	def in_bounds(self, point):
		return point[0] >= 0 and point[0] < len(self.grid) and point[1] >= 0 and point[1] < len(self.grid[0])

	def occupancy_between(self, start, end):

		num_squares = 0

		angle = math.atan2(end[1] - start[1], end[0] - start[0])

		for i in range(round(math.dist(start, end))):

			n_point = [0, 0]
			n_point[0] = round(i * math.cos(angle) + start[0])
			n_point[1] = round(i * math.sin(angle) + start[1])

			if self.inBounds(n_point) and self.grid[n_point[0]][n_point[1]] > 0:
				return True

		return False


	def label_map_for_goal(self, goal_point):
		oob_right = False
		oob_left = False
		oob_top = False
		oob_bottom = False

		ray_len = 0


	def to_msg(self):
		n_msg = OccupancyGrid()

		n_msg.header.stamp = rospy.Time.now()
		n_msg.header.frame_id = 'map'

		n_msg.info.resolution = self.resolution
		n_msg.info.width = self.width
		n_msg.info.height = self.height
		n_msg.info.origin.orientation.w = 1.0

		n_msg.data = []

		for row, val in enumerate(self.grid): 
			for col, measured_val in enumerate(val):
				n_msg.data.append(measured_val)

		return n_msg

class Follower:

	def __init__(self):
		self.updated_laser = False
		self.updated_force = False

		self.map = OccupancyGridMap2D()
		
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_cb)
		self.force_sub = rospy.Subscriber('/force_topic', WrenchStamped, self.force_cb)

		self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
		self.traj_pub = rospy.Publisher('/path', Path, queue_size=1)

		self.saved_force = None
		self.saved_laser = None

		self.traj = Path()

	def laser_cb(self, data):

		if not self.updated_laser:
			self.saved_laser = data

			self.updated_laser = True

	def force_cb(self, data):

		if not self.updated_force:
			self.saved_force = data

			self.updated_force = True

	def pursue_trajectory(self):
		pass

	def cmd_vel_loop(self):
		pass

	def wait_until_updated(self):
		while not self.updated_force and not self.updated_laser:
			rospy.spin_once()

	def update_map(self):
		self.map.init_from_scan(self.saved_laser)

	def publish_map(self):
		self.map_pub.publish(self.map.to_msg())

	def publish_trajectory(self):
		self.traj_pub.publish(self.traj)

	def get_goal_point_from_force(self):
		
		# normalize the force vector to half the side length of the map as long as it's above the threshold value
		# only considering the force for now, and discarding the z value to place it on a 2d plane
		vector_norm = ( self.saved_force.wrench.force.x ** 2 + self.saved_force.wrench.force.y ** 2 + self.saved_force.wrench.force.z ** 2) ** 2

		normalized_x_component = self.saved_force.wrench.force.x * ( .5 * self.map.width ) / vector_norm
		normalized_y_component = self.saved_force.wrench.force.y * ( .5 * self.map.width ) / vector_norm

		# get the end point of the vector
		return [ int(normalized_x_component), int(normalized_y_component) ]

	def get_trajectory(self, start_point, end_point):
		
		to_ret = Path()

		# just this for now
		pose1 = PoseStamped()
		pose1.header.frame_id = 'map'
		pose1.pose.position.x = start_point[0]
		pose1.pose.position.y = start_point[1]
		pose1.pose.position.z = 0
		pose1.pose.orientation.w = 1

		pose2 = PoseStamped()
		pose2.header.frame_id = 'map'
		pose2.pose.position.x = goal_point[0]
		pose2.pose.position.y = goal_point[1]
		pose2.pose.position.z = 0
		pose2.pose.orientation.w = 1

		to_ret.poses.append(pose1)
		to_ret.poses.append(pose2)

		return to_ret


	def tick(self):

		self.updated_laser = False
		self.updated_force = False
		
		self.wait_until_updated()

		self.update_map()

		goal_point = self.get_goal_point_from_force()

		self.traj = self.get_trajectory([0, 0], goal_point)

		self.publish_map()
		self.publish_trajectory()


if __name__ == '__main__':

	rospy.init("follower_node", anonymous=True)

	follower_obj = Follower()

	while rospy.ok():
		follower_obj.tick()
#!/usr/bin/python3

import rospy

class OccupancyGridMap2D:

	def __init__(self, point_cloud):
		self.grid = []
		self.cost_grid = []
		self.util_grid = []

		self.width = 200
		self.height = 200
		self.resolution = .05

		self.initFromVelodyne(point_cloud)

	def resetMap(self):
		for row in self.grid:
			for col in range(len(row)):
				row[col] = 0

	def resetCosts(self):
		for row in self.cost_grid:
			for col in range(len(row)):
				row[col] = 0

	def initCosts(self, robot_positions):
		for row in range(len(self.cost_grid)):
			for col in range(len(self.cost_grid[row])):
				if [row, col] in robot_positions:
					self.cost_grid[row][col] = 0
				else:
					self.cost_grid[row][col] = 1000000

	def initUtils(self):
		for row in range(len(self.cost_grid)):
			for col in range(len(self.cost_grid[row])):
				self.util_grid[row][col] = 0

	def initFromVelodyne(self, point_cloud):

		self.resetMap()



	def inBounds(self, point):
		return point[0] >= 0 and point[0] < len(self.grid) and point[1] >= 0 and point[1] < len(self.grid[0])

	def occupancyBetween(self, start, end):

		num_squares = 0

		angle = math.atan2(end[1] - start[1], end[0] - start[0])

		for i in range(round(math.dist(start, end))):

			n_point = [0, 0]
			n_point[0] = round(i * math.cos(angle) + start[0])
			n_point[1] = round(i * math.sin(angle) + start[1])

			if self.inBounds(n_point) and self.grid[n_point[0]][n_point[1]] > 0:
				return True

		return False

	def toMsg(self):
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
		pass

	def laser_cb(self):
		pass

	def force_cb(self):
		pass

	def pursue_trajectory(self):
		pass

	def cmd_vel_loop(self):
		pass

	def wait_until_updated(self):
		while not self.updated_force and not self.updated_laser:
			pass

	def update_map(self):
		pass

	def publish_map(self):
		pass

	def publish_trajectory(self):
		pass

	def get_goal_point_from_force(self):
		pass

	def get_trajectory(self, start_point, end_point):
		pass

	def get_force_vector(self):
		pass

	def tick(self):
		
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
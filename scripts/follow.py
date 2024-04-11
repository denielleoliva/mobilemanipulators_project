#!/usr/bin/python3

import rospy

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
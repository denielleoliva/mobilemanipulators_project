#!/usr/bin/python3

import copy

import rospy

from geometry_msgs.msg import WrenchStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan

def get_smallest_dist_and_direction(ang1, ang2):

    temp1 = ang1
    temp2 = ang2

    if ang1 <= 0:
        temp1 = math.pi * 2 + ang1

    if ang2 <= 0:
        temp2 = math.pi * 2 + ang2

    distance1 = temp1 - temp2
    distance2 = temp2 - temp1

    if distance1 < 0:
        distance1 += math.pi * 2

    if distance2 < 0:
        distance2 += math.pi * 2

    if distance1 >= distance2:
        return distance2, 'left'
    else:
        return distance1, 'right'

class Follower:

	FORCE_THRESHOLD = 1.5

	def __init__(self):
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_cb)
		self.force_sub = rospy.Subscriber('/force_topic', WrenchStamped, self.force_cb)

		self.cmd_vel_pub = rospy.Publisher('/bvr_sim/cmd_vel', Twist, queue_size=10)

		self.saved_force = None
		self.obstacles = []

	def update_obstacles_from_laser(self, data):
		n_obstacles = []

		for index, val in enumerate(data.ranges):
			if val < data.range_min or val > data.range_max:
				continue

			angle = data.angle_min + index * angle_increment

			n_x = val * math.cos(angle)
			n_y = val * math.sin(angle)

			n_obstacles.append(np.array([n_x, n_y]))

		self.obstacles = n_obstacles

	def laser_cb(self, data):
		self.update_obstacles_from_laser(data)

	def force_cb(self, data):
		self.saved_force = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

		self.transform_force()

	def calc_overall_force(self):

		total_force = copy.deepcopy(self.saved_force)
		current_obstacles = copy.deepcopy(self.obstacles) 

		# equations for attractive and repulsive force are taken from here: https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
		# max distance is one meter
		q_star = 1.0
		gains = np.array([.1, .1])

		for i in current_obstacles:
			distance = np.linalg.norm(i)

			if distance > q_star:
				continue

			# I think the sign here is correct but it's questionable
			total_force +=  gains * ( (1 / q_star) - ( 1 / distance ) ) * ( 1 / distance ** 2 )

		return total_force

	def wait_for_initial_scans(self):
		while self.saved_force == None or self.obstacles == []:
			pass

	def get_twist_from_force(self, vel, delta_t=.2):

		true_angle = math.atan2(vel[1], vel[0])

		dist, direction = get_smallest_dist_and_direction( 0, true_angle )

		twist = Twist()

		twist.linear.x = max(np.linalg.norm(vel) * math.cos(dist), 0)

		if math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2) == 0:
			twist.angular.z = 0
		elif direction == 'right':
			twist.angular.z = -1 * dist / delta_t
		else:
			twist.angular.z = dist / delta_t

		return twist

	def tick(self):

		if np.linalg.norm(self.saved_force) < FORCE_THRESHOLD:
			return

		total_force = self.calc_overall_force()

		twist = self.get_twist_from_force(total_force)

		self.cmd_vel_pub.publish(twist)


if __name__ == '__main__':

	rospy.init("follower_node", anonymous=True)

	follower_obj = Follower()

	follower_obj.wait_for_initial_scans()

	while rospy.ok():
		follower_obj.tick()
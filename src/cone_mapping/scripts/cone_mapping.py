#!/usr/bin/env python3

from inspect import signature
import rospy
import math
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Pose2D, PoseArray, Pose


class ConeMapper:
	def __init__(self):
		#            [Not used  , Start/Orange, Right/Yellow, Left/Blue]
		self.cones = [PoseArray(), PoseArray(), PoseArray(), PoseArray()]
		self.position = PoseStamped()

		# Subscribers
		self.sub_pose = rospy.Subscriber(
			"/pose_stamped", PoseStamped, callback=self.pose_callback
		)
		self.sub_cone = rospy.Subscriber(
			"/sensor_fusion/output", Float32MultiArray, callback=self.cone_callback
		)

		# Publishers
		self.pub_right = rospy.Publisher(
			"/cone_right", PoseArray, latch=True, queue_size=1
		)
		self.pub_left = rospy.Publisher(
			"/cone_left", PoseArray, latch=True, queue_size=1
		)
		self.pub_start = rospy.Publisher(
			"/cone_orange", PoseArray, latch=True, queue_size=1
		)

	def pose_callback(self, msg: PoseStamped):
		self.position = msg

	def insert_cone(self, cone, cone_list_index):
		'''
		This function apply an identity function to the cone.
		If the new cone is at distance d from an already discovered one, and d < distance_threshold,
		the new cone is not inserted in the list of cones.
		'''
		if cone_list_index == 0 or cone_list_index == 1:
			cone_list_index = 1

		cone_list = self.cones[cone_list_index]
		distance_threshold = 2
		found = False
		rospy.loginfo("Analyzing detected cone {}, {}".format(cone.position.x, cone.position.y))

		# cone_left = self.cones[3]
		# cone_right = self.cones[2]
		# if cone_left != None and cone_right != None:
		# 	if len(cone_left.poses) > 2 and len(cone_right.poses) > 2:
		# 		distance1 = math.dist([cone.position.x, cone.position.y], [cone_right.poses[-1].position.x, cone_right.poses[-1].position.y])
		# 		distance2 = math.dist([cone.position.x, cone.position.y], [cone_right.poses[-2].position.x, cone_right.poses[-2].position.y])
		# 		distance3 = math.dist([cone.position.x, cone.position.y], [cone_left.poses[-1].position.x, cone_left.poses[-1].position.y])
		# 		distance4 = math.dist([cone.position.x, cone.position.y], [cone_left.poses[-2].position.x, cone_left.poses[-2].position.y])

		# 		if distance1 < distance3 and distance2 < distance4 and cone_list_index == 3:
		# 			return
		# 		if distance3 < distance1 and distance4 < distance2 and cone_list_index == 2:
		# 			return

		# for each cone in the cone list, compute the distance between the new cone and old cones and check if we should add
		for idx, old_cone in enumerate(cone_list.poses):
			distance = math.dist([cone.position.x, cone.position.y], [old_cone.position.x, old_cone.position.y])

			if distance < distance_threshold:
				rospy.loginfo("Alredy seen cone {}, {}".format(old_cone.position.x, old_cone.position.y))
				found = True
				break
		
		# if the cone is new, insert it in the cone list
		if not found:
			rospy.loginfo("New cone {}, {}".format(cone.position.x, cone.position.y))
			cone_list.poses.append(cone)

		return cone_list

	def cone_callback(self, msg: Float32MultiArray):
		axis2index = {"x": 0, "y": 1, "z": 2, "c": 3}
		noise_threshold = 3

		# read each cone's position
		for i in range(0, len(msg.data), 4):
			x = msg.data[i + axis2index["x"]]
			y = msg.data[i + axis2index["y"]]

			# filter the object if it's too distant
			if x <= 0 or x > noise_threshold or y == 0 or abs(y) > noise_threshold:
				continue

			cone = Pose()
			cone.position.x = self.position.pose.position.x + x
			cone.position.y = self.position.pose.position.y + y

			color = int(msg.data[i + axis2index["c"]])
			if color < 0:
				return

			if color == 0 or color == 1:
				self.cones[1] = self.insert_cone(cone, 1)
				# cone_list = self.cones[1]
				# cone_list.poses.append(cone)
			else:
				self.cones[color] = self.insert_cone(cone, color)
				# cone_list = self.cones[color]
				# cone_list.poses.append(cone)


def main():
	rospy.init_node("cone_mapping")

	print("Start node cone_mapping")
	cone_mapper = ConeMapper()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		cone_mapper.pub_left.publish(cone_mapper.cones[3])
		cone_mapper.pub_right.publish(cone_mapper.cones[2])
		cone_mapper.pub_start.publish(cone_mapper.cones[1])
		rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Pose2D, PoseArray, Pose


class ConeMapper():
    def __init__(self):
        self.cone_left = PoseArray()
        self.cone_right = PoseArray()
        self.current_pos = PoseStamped()

        # subscribers
        self.sub_pose = rospy.Subscriber("/pose_stamped", PoseStamped, callback=self.pose_callback)
        self.sub_cone = rospy.Subscriber("/model/lidar/output", Float32MultiArray, callback=self.cone_callback)

        # publishers
        self.pub_right = rospy.Publisher("/cone_right", PoseArray, latch=True, queue_size=1)
        self.pub_left = rospy.Publisher("/cone_left", PoseArray, latch=True, queue_size=1)


    def pose_callback(self, msg: PoseStamped):
        self.current_pos = msg


    def cone_callback(self, msg: Float32MultiArray):
        cone2idx = {"x": 0, "y": 1, "z": 2}
        thresh_noise = 3

        # read each cone's position
        for i in range(0, len(msg.data), 3):
            x = msg.data[i+cone2idx["x"]]
            y = msg.data[i+cone2idx["y"]]

            cone_pos = Pose()
            cone_pos.position.x = self.current_pos.pose.position.x + x
            cone_pos.position.y = self.current_pos.pose.position.y + y

            # filter the object if it's too distant
            if cone_pos.position.x > self.current_pos.pose.position.x + thresh_noise or \
               cone_pos.position.x < self.current_pos.pose.position.x - thresh_noise or \
               cone_pos.position.y > self.current_pos.pose.position.y + thresh_noise or \
               cone_pos.position.y < self.current_pos.pose.position.y - thresh_noise or \
               x <= 0:
                continue

            # distribute cone on left or right
            if y > 0:
                self.cone_right.poses.append(cone_pos)
            elif y < 0:
                self.cone_left.poses.append(cone_pos)


def main():
    cone_mapper = ConeMapper()
    rospy.init_node("cone_mapping")
    print("Start node cone_mapping")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        cone_mapper.pub_right.publish(cone_mapper.cone_right)
        cone_mapper.pub_left.publish(cone_mapper.cone_left)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

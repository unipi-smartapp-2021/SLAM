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
        cone2idx = {"theta": 0, "phi": 1, "magnitude": 2}

        for i in range(0, len(msg.data), 3):
            x = msg.data[i+cone2idx["magnitude"]] * math.cos(msg.data[i+cone2idx["theta"]])
            y = msg.data[i+cone2idx["magnitude"]] * math.sin(msg.data[i+cone2idx["theta"]])

            cone_pos = Pose()
            cone_pos.position.x = self.current_pos.pose.position.x + x
            cone_pos.position.y = self.current_pos.pose.position.y + y

            # TODO need to fix the distribution
            # distribute cone on left or right
            if msg.data[i+cone2idx["theta"]] > 0:
                self.cone_right.poses.append(cone_pos)
            else: 
                self.cone_left.poses.append(cone_pos)


def main():
    cone_mapper = ConeMapper()
    rospy.init_node("cone_mapping")
    print("Start node cone_mapping")
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        cone_mapper.pub_right.publish(cone_mapper.cone_right)
        cone_mapper.pub_left.publish(cone_mapper.cone_left)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

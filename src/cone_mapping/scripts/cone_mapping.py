#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Pose2D, PoseArray, Pose


class ConeMapper:
    def __init__(self):
        #            [   Left    ,   Right    ]
        self.cones = [PoseArray(), PoseArray()]
        self.position = PoseStamped()

        # Subscribers
        self.sub_pose = rospy.Subscriber(
            "/pose_stamped", PoseStamped, callback=self.pose_callback
        )
        self.sub_cone = rospy.Subscriber(
            "/model/lidar/output", Float32MultiArray, callback=self.cone_callback
        )

        # Publishers
        self.pub_right = rospy.Publisher(
            "/cone_right", PoseArray, latch=True, queue_size=1
        )
        self.pub_left = rospy.Publisher(
            "/cone_left", PoseArray, latch=True, queue_size=1
        )

    def pose_callback(self, msg: PoseStamped):
        self.position = msg

    def cone_callback(self, msg: Float32MultiArray):
        axis2index = {"x": 0, "y": 1, "z": 2}
        noise_threshold = 3

        # read each cone's position
        for i in range(0, len(msg.data), 3):
            x = msg.data[i + axis2index["x"]]
            y = msg.data[i + axis2index["y"]]

            # filter the object if it's too distant
            if x <= 0 or x > noise_threshold or y == 0 or abs(y) > noise_threshold:
                continue

            cone = Pose()
            cone.position.x = self.position.pose.position.x + x
            cone.position.y = self.position.pose.position.y + y
            self.cones[y < 0].poses.append(cone)


def main():
    rospy.init_node("cone_mapping")

    print("Start node cone_mapping")
    cone_mapper = ConeMapper()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        cone_mapper.pub_left.publish(cone_mapper.cones[0])
        cone_mapper.pub_right.publish(cone_mapper.cones[1])
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

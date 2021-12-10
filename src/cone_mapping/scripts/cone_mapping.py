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

    def insert_cone(self, cone, cone_list_index):
        '''
        This function apply tries to apply an identity function to the cone.
        If the new cone is at distance d from an already discovered one, and d < distance_threshold,
        the new cone is not inserted in the list of cones.
        '''
        # get the current cone list
        cone_list = self.cones[cone_list_index]
        # distace threshold
        distance_threshold = 2
        found = False

        rospy.loginfo("Analying detected cone {}, {}".format(cone.position.x, cone.position.y))


        # for each cone in the cone list
        for idx, old_cone in enumerate(cone_list.poses):
            # compute the distance between the new cone and the cone in the list
            distance = math.dist([cone.position.x, cone.position.y], [old_cone.position.x, old_cone.position.y])
            # if there is already a cone in the area of the new detected cone (its distance form the old cone is below the threshold)
            # break the loop and do not insert the new cone in the cone list
            if distance < distance_threshold:
                rospy.loginfo("Alredy seen cone {}, {}".format(old_cone.position.x, old_cone.position.y))
                found = True
                break
        
        # if i cannot find any cone in the nearby of the new detected cone, insert the new cone in the cone list
        if not found:
            rospy.loginfo("New cone {}, {}".format(cone.position.x, cone.position.y))
            cone_list.poses.append(cone)

        return cone_list

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

            self.cones[y < 0] = self.insert_cone(cone, (y < 0))

            #self.cones[y < 0].poses.append(cone)

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

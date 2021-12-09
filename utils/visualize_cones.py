import matplotlib.pyplot as plt
import numpy as np
import rosbag
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Float32MultiArray

bag = rosbag.Bag('cone_right_pose.bag')
cone_x, cone_y = [], []
car_x, car_y = [], []

for topic, msg, t in bag.read_messages(topics=['/cone_right', '/pose_stamped']):
    if topic == "/cone_right":
        for pose in msg.poses:
            cone_x.append(pose.position.x-1.5)
            cone_y.append(pose.position.y-0.21)
    elif topic == "/pose_stamped":
        car_x.append(msg.pose.position.x)
        car_y.append(msg.pose.position.y)

# for topic, msg, t in bag.read_messages(topics=['/model/lidar/output']):
#     for i in msg.data:
#         if i < 0:
#             print(i)


plt.scatter(car_x, car_y, color='r')
plt.scatter(cone_x, cone_y, color='b')
plt.show()

bag.close()
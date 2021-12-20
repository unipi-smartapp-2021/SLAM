import matplotlib.pyplot as plt
import numpy as np
import rosbag
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Float32MultiArray
import sys

try:
    bag = rosbag.Bag(sys.argv[1])
except:
    bag = rosbag.Bag('l_filtered.bag')

# bag = rosbag.Bag('cone_right_pose.bag')
conel_x, conel_y = [], []
coner_x, coner_y = [], []
coneo_x, coneo_y = [], []
car_x, car_y = [], []

for topic, msg, t in bag.read_messages(topics=['/cone_right', '/cone_left', '/cone_orange', '/pose_stamped']):
    if topic == "/cone_right":
        for pose in msg.poses:
            coner_x.append(pose.position.x)
            coner_y.append(pose.position.y)
    elif topic == "/cone_left":
        for pose in msg.poses:
            conel_x.append(pose.position.x)
            conel_y.append(pose.position.y)
    elif topic == "/cone_orange":
        for pose in msg.poses:
            coneo_x.append(pose.position.x)
            coneo_y.append(pose.position.y)
    elif topic == "/pose_stamped":
        car_x.append(msg.pose.position.x)
        car_y.append(msg.pose.position.y)

car_x.append(1)
car_x.append(1)
car_y.append(-5)
car_y.append(5)

# plt.plot(coner_x, [np.mean(conel_y) for _ in range(len(coner_y))])
# plt.plot(coner_x, [np.mean(coner_y) for _ in range(len(coner_y))])
plt.scatter(car_x, car_y, color='r')
plt.scatter(conel_x, conel_y, color='b')
plt.scatter(coner_x, coner_y, color='y')
plt.scatter(coneo_x, coneo_y, color='tab:orange')
plt.show()

bag.close()
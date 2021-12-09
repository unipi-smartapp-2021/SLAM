import matplotlib.pyplot as plt
import numpy as np
import rosbag
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Float32MultiArray

bag = rosbag.Bag('sensory.bag')
# bag = rosbag.Bag('cone_right_pose.bag')
cone_x, cone_y = [], []
cone2idx = {"x": 0, "y": 1, "z": 2}
i = 0

for topic, msg, t in bag.read_messages(topics=['/model/lidar/output']):

    for i in range(0, len(msg.data), 3):
        x = msg.data[i+cone2idx["x"]] + i
        y = msg.data[i+cone2idx["y"]]

        if x <= 0:
            continue

        cone_x.append(x)
        cone_y.append(y)
        i += 1

# conel_x.append(1)
# conel_x.append(1)
# conel_y.append(-20)
# conel_y.append(20)

# for topic, msg, t in bag.read_messages(topics=['/model/lidar/output']):
#     for i in msg.data:
#         if i < 0:
#             print(i)

plt.scatter(cone_x, cone_y, color='r')
plt.show()

bag.close()
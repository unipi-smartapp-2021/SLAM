#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped, Pose2D, PoseArray, Pose
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.left_plot, = plt.plot([], [], 'bo')
        self.right_plot, = plt.plot([], [], 'yo')
        self.orange_plot, = plt.plot([], [], 'o', color="#ffa444")
        self.car_plot, = plt.plot([], [], 'ro')
        self.left_x, self.left_y = [] , []
        self.right_x, self.right_y = [] , []
        self.orange_x, self.orange_y = [] , []
        self.car_x, self.car_y = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 50)
        self.ax.set_ylim(-5, 5)
        self.ax.invert_yaxis()
        return self.left_plot
    
    def left_cone_callback(self, msg):
        self.left_x = ([cone.position.x for cone in msg.poses])
        self.left_y = ([cone.position.y for cone in msg.poses])

    def right_cone_callback(self, msg):
        self.right_x = ([cone.position.x for cone in msg.poses])
        self.right_y = ([cone.position.y for cone in msg.poses])
    
    def orange_cone_callback(self, msg):
        self.orange_x = ([cone.position.x for cone in msg.poses])
        self.orange_y = ([cone.position.y for cone in msg.poses])

    def car_position_callback(self, msg):
        self.car_x.append(msg.pose.position.x)
        self.car_y.append(msg.pose.position.y)

    def update_plot(self, frame):
        self.left_plot.set_data(self.left_x, self.left_y)
        self.right_plot.set_data(self.right_x,self.right_y)
        self.orange_plot.set_data(self.orange_x,self.orange_y)
        self.car_plot.set_data(self.car_x,self.car_y)
        return self.left_plot


rospy.init_node('realtime_visualizer')
vis = Visualiser()
sub = rospy.Subscriber('/cone_left', PoseArray, vis.left_cone_callback)
sub = rospy.Subscriber('/cone_right', PoseArray, vis.right_cone_callback)
sub = rospy.Subscriber('/cone_orange', PoseArray, vis.orange_cone_callback)
sub = rospy.Subscriber('/pose_stamped', PoseStamped, vis.car_position_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 

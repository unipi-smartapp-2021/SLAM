# SLAM
SLAM implementation by using localization module `laser_scan_matcher` and mapping the cones relative position using the absolute position `pose_stamped` outputted by `laser_scan_matcher`.

<!-- 
The slam toolbox package can be downloaded at the following link [here](https://github.com/SteveMacenski/slam_toolbox)

The provided toolbox build the map using `sensor_msgs::LaserScan` instead the LIDAR give us back data of type `sensor_msgs::PointCloud`.

To cope with this, we transform the point cloud into laser scan using the provided package [here]( http://wiki.ros.org/pointcloud_to_laserscan)

The slam toolbox look for LaserScan messages on the topic specified in `slam_toolbox/config` in the param *scan_topic*.
 -->

This is a brief guide on how to assembly the various components:
```
git clone https://github.com/unipi-smartapp-2021/SLAM
cd SLAM
git submodule update --init --recursive
sudo apt-get install libgsl0-dev
```

If catkin_make fails due to missing `csm` package, install it:
```
cd src
git clone https://github.com/AndreaCensi/csm
```

Overwrite the following files (modified the topic in the source code)
```
cp utils/pointcloud_to_laserscan_nodelet.cpp src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_nodelet.cpp
cp utils/sample_node.launch src/pointcloud_to_laserscan/launch/sample_node.launch
cp utils/laser_scan_matcher.cpp src/scan_tools/laser_scan_matcher/src/laser_scan_matcher.cpp
```

Try and pray that everything builds
```
catkin_make
```

## Example
Follow the order to launch each line
```
roscore
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node
rosrun cone_mapping cone_mapping.py
rosparam set use_sim_time true

# launch the bag, localization and set the simulation clock
rosbag play <bag> --clock
rosrun laser_scan_matcher laser_scan_matcher_node

# print the output topics
rostopic echo /pose_stamped
rostopic echo /cone_right
rostopic echo /cone_left
```

If you want to visualize the plotting of cones, create a bag recording the topic `/cone_left /cone_right /pose_stamped`, then launch the `utils/visualize_cones.py`. Example

![](imgs/track.png)

# TODO
- [] Make launchfile 
- [] Use directly Pointercloud instead of converting to Laserscan
- [] Test on the simulator

<!-- 
## slam-toolbox

**IMPORTANT** before doing anything change the branch to `noetic-devel`

Install dependencies with `rosdep install -q -y -r --from-paths src --ignore-src`

Install `apt install ros-noetic-slam-toolbox` if required.


## pointcloud-to-laserscan

**IMPORTANT** before doing anything change the branch to `lunar-devel`

Notice that `geometry2` is required to build this package. -->

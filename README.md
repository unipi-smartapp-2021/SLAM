# SLAM
SLAM implementation

## SLAM package

The slam toolbox package can be downloaded at the following link [here](https://github.com/SteveMacenski/slam_toolbox)

The provided toolbox build the map using `sensor_msgs::LaserScan` instead the LIDAR give us back data of type `sensor_msgs::PointCloud`.

To cope with this, we transform the point cloud into laser scan using the provided package [here]( http://wiki.ros.org/pointcloud_to_laserscan)

The slam toolbox look for LaserScan messages on the topic specified in `slam_toolbox/config` in the param *scan_topic*.


## slam-toolbox

**IMPORTANT** before doing anything change the branch to `noetic-devel`

Install dependencies with `rosdep install -q -y -r --from-paths src --ignore-src`

Install `apt install ros-noetic-slam-toolbox` if required.


## pointcloud-to-laserscan

**IMPORTANT** before doing anything change the branch to `lunar-devel`

Notice that `geometry2` is required to build this package.
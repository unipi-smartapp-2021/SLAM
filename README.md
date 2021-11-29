# SLAM
SLAM implementation

## SLAM package

The slam toolbox package can be downloaded at the following link [here].
The provided toolbox build the map using 
'''
sensor_msgs::LaserScan
'''
The LIDAR give us back data of type
'''
sensor_msgs::PointCloud
'''
To cope with this, we transform the point cloud into laser scan using the provided package [here2].

The slam toolbox look for LaserScan messages on the topic specified in 
'''
slam_toolbox/config
'''
in the param *scan_topic*.





[here]: https://github.com/SteveMacenski/slam_toolbox

[here2]: http://wiki.ros.org/pointcloud_to_laserscan

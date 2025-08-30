# pc2_to_laserscan

ROS2 package (C++) that subscribes to `sensor_msgs/PointCloud2`, converts it into a `sensor_msgs/LaserScan`, and publishes the scan.

## Features
- C++ node: `pc2_to_laserscan_cpp`
- All parameters configurable via `config/params.yaml`
- Launch file: `launch/pc2_to_laserscan_launch.py`


## Installation and build
```bash
# in ROS2 workspace src
git clone <this_package> 
cd ..
colcon build
source install/setup.bash
ros2 launch pc2_to_laserscan pc2_to_laserscan_launch.py
```

## Parameters

All parameters are configured in [`config/params.yaml`](config/params.yaml).  
They are loaded automatically by the launch file.

| Parameter         | Type    | Default     | Description |
|-------------------|---------|-------------|-------------|
| `input_cloud`     | string  | `/points`   | Topic name of the input `sensor_msgs/PointCloud2`. |
| `output_laserscan`| string  | `/scan`     | Topic name where the resulting `sensor_msgs/LaserScan` is published. |
| `angle_min`       | double  | `-1.5708`   | Minimum angle of the scan (radians). |
| `angle_max`       | double  | `1.5708`    | Maximum angle of the scan (radians). |
| `angle_increment` | double  | `0.0058`    | Angular resolution of the scan (radians per step). |
| `z_min`           | double  | `-0.2`      | Minimum z-height of points (below this, points are ignored). |
| `z_max`           | double  | `1.0`       | Maximum z-height of points (above this, points are ignored). |
| `range_min`       | double  | `0.05`      | Minimum valid range (meters). |
| `range_max`       | double  | `50.0`      | Maximum valid range (meters). |
| `use_inf`         | bool    | `true`      | If `true`, empty bins are filled with `inf`. If `false`, empty bins become `NaN`. |
| `scan_time`       | double  | `0.1`       | Approximate time between scans (seconds). |

### Editing parameters
To modify parameters, edit the YAML file:

```yaml
pc2_to_laserscan_cpp:
  ros__parameters:
    input_cloud: "/velodyne_points"
    output_laserscan: "/velodyne_scan"
    angle_min: -3.14
    angle_max: 3.14
    angle_increment: 0.005
    z_min: -1.0
    z_max: 2.0
    range_min: 0.1
    range_max: 80.0
    use_inf: true
    scan_time: 0.1


## Demo Video

You can watch a short demo recorded in RViz showing the point cloud to laser scan conversion:

[![Demo Video](data/laserscan.png)](data/laserscan_video.webm)



# off_highway_sensor_drivers_examples

The off_highway_sensor_drivers_examples package contains sample launch files and scripts to assist
with further processing of the data provided by the off_highway_sensor_drivers.

## off_highway_premium_radar_sample_filter

Series-connected Point Cloud Library (PCL) PassThrough filter nodes, which filter the point cloud
returned by the off_highway_premium_radar_sample driver by the z, x and radar_cross_section field
values of the locations. Thresholds are set to example values directly in the launch file and can be
adapted at runtime using the dynamic reconfigure plugin of rqt.

### Build

> :warning: **Running this example currently requires building perception_pcl from source.**
> :warning:

Since the currently available ROS 2 binaries of the pcl_ros package do not yet contain all required
features to run this example, the [perception_pcl](https://github.com/ros-perception/perception_pcl)
repository needs to be cloned and built from source. To accomplish this, `cd` into the source folder
of your workspace and execute

```bash
git clone -b ros2 https://github.com/ros-perception/perception_pcl
git -C perception_pcl checkout c1ccdd2776043ba1477926acceab7df75f64bf22
cd ..
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
```

to build it. Source the workspace before running the example with

```bash
ros2 launch off_highway_sensor_drivers_examples off_highway_premium_radar_sample_filter_launch.py
```

### Launch files

* **[off_highway_premium_radar_filter_launch.py](launch/off_highway_premium_radar_filter_launch.py)**:
  Starts the off_highway_premium_radar_sample driver with the given parameters, PCL PassThrough
  filters and rviz.
  * Arguments:
    * **off_highway_premium_radar_params**: Path to ROS YAML parameter file to load for driver. If
      not provided, default parameters from the off_highway_premium_radar_sample package are loaded.
    * **rviz_config**: Path to rviz configuration file. If not provided, default configuration is
      loaded from this package.

## off_highway_radar_extract_velocity

Example to demonstrate, how velocity information from messages other than
`geometry_msgs::msg::TwistStamped` can be passed to the off_highway_radar driver. The
extract_velocity component extracts the velocity information from a `nav_msgs::msg::Odometry`
message and republishes it as `geometry_msgs::msg::TwistStamped` message.

### Launch files

* **[off_highway_radar_extract_velocity_launch.py](launch/off_highway_radar_extract_velocity_launch.py)**:
  Starts the extract_velocity component and the sender component of the off_highway_radar in
  a composition to make use of intra-process communication.
  * Arguments:
    * **params**: Path to ROS YAML parameter file to load for sender component of
      off_highway_radar. If not provided, default parameters from the off_highway_radar package are
      loaded.

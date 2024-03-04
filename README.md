# Waypoint follower tools

This package offers some tools, which can improve the claudi_gazebo simulation, in particular the waypoint follower. 

## Description

The package offers these tools:
* Ground truth publisher
* Path plotter
* Waypoint generator
* GPS error simulator
* Wait for localization node
* Odom Ros2 republisher (deprecated)

### Ground truth publisher
The launch file creates a Gazebo-Ros2 bridge for the tf_message that describes the world-robot relation using the `ros_gz_bridge` package. In addition, it stats the ground_truth_publisher_node, which listens to the ground truth tf topic, extracts the claudi/base_link frame and publishes the pose as a PoseStamped message relative to the map frame.

> When using the gazebo bridge to forward the tf message, the message consists of many transformations, all with empty headers. To extract the claudi/base_link transformation, the node simply chooses the first transformation. For now, this is consistently the right transformation but this might change in the future.

### Path plotter
There are two path plotter nodes, one for the ground truth position and one for the localization position.
These nodes listen to the corresponding topic, `pose/base_link` and `odometry/global`, respectively, and create a path message which is then published in a reduced frequency to `path/ground_truth` and `path/odometry/global`.

The launch file stats both nodes.

### Waypoint generator
The waypoint generator can generate waypoints with variable separations in different patterns:

Line: All waypoints in a staight line. Waypoint separation and accumulated distance between waypoints can be configured.

Cosine: The waypoints follow a cosine form. Waypoint separation and distance can be configured. Distance can either be set to the accumulated distance between waypoints or to the distance in propagation direction.

Square wave: not yet implemented

### GPS error simulator
todo

### Wait for localization node
This node is just listening to the map topic. After the first message is received, the node shuts down. This allows to start the navigation as soon as the localization is ready by using the OnProcessExit event handler.

### Odom Ros2 republisher
> Not required anymore, as cpp node included in claudi_gazebo.

Node, that listens to `/odom/gazebo` (ros_gz_bridge output), adjusts header.frame_id and child_frame_id, before republishing to `/odom`.


## Getting Started

### Dependencies

* For the waypoint generator, a running gps localization with navsat_transform_node is required
* ... many packages, use rosdep to install

### Installing

Clone, build and use rosdep to install dependencies.

### Executing program

#### Ground truth publisher
```bash
ros2 launch wpf_tools ground_truth_publisher.launch.py
```

#### Path plotter
To start both nodes (default), use the launch file:
```bash
ros2 launch wpf_tools path_plotter.launch.py
```

To select only one path plotter node, you can either use a launch argument or just run the relevant node:
```bash
ros2 launch wpf_tools path_plotter.launch.py path_localization_enabled:=True path_ground_truth_enabled:=True
```
```bash
ros2 run wpf_tools path_plotter_localization
```
```bash
ros2 run wpf_tools path_plotter_ground_truth
```

#### Waypoint generator
Adjust the yaml config file or create a new one.
```bash
ros2 run wpf_tools waypoint_generator_node.py --ros-args --param-file <package_src_or_share>/wpf_tools/config/wp_generator.yaml
```
```bash
ros2 run wpf_tools waypoint_generator_node.py --ros-args --param-file ./src/wpf_tools/config/wp_generator.yaml
```

### GPS error simulator
todo


### Topics

#### Ground truth publisher
* `/pose/base_link`

#### Path plotter
* `/path/localization`
* `/path/odometry/global`

#### Waypoint generator
* None

#### GPS error simulator
* `/emlid/modified`


## Authors

[Nils Friedrich](mailto:nils-jonathan.friedrich@student.tu-freiberg.de)

## Version History

* 0.1
    * Initial Release
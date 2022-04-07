# ARTI Move Base Tutorials

Packages to get you started with [ARTI Move Base](https://github.com/ARTI-Robots/arti_move_base).


## Prerequisites

- [ROS Kinetic](http://wiki.ros.org/kinetic/) or [Melodic](http://wiki.ros.org/melodic/)
- ARTI Move Base: https://github.com/ARTI-Robots/arti_move_base
- ARTI Stanley Control Path Follower: https://github.com/ARTI-Robots/stanley_control_path_follower
- teb_local_planner: `sudo apt install ros-$(rosversion -d)-teb-local-planner`
- turtlesim: `sudo apt install ros-$(rosversion -d)-turtlesim`


## Usage

Launch the example:

```shell
roslaunch arti_move_base_example arti_move_base_example.launch
```

Send a goal pose using the "2D Nav Goal" RViz tool. The turtle should now follow a path to the goal pose.


## Troubleshooting

Check that scripts/odom_republisher.py is executable.

## Synopsis

This Simple ROS package provides a quick headstart for testing high level path planning / visual servoing algorithms on multiple fixed-wing unmanned aerial vehicles. This package requires Gazebo Simulation environment.

## Installation

Just place this ROS package under the src folder of your catkin workspace before running `catkin_make`

(Tested with ROS-Indigo and Gazebo 2.3) -- Tested OK

## Example

Run the following commands in different terminal windows

```$ roslaunch simple_airplane_gazebo simple_airplane_empty_world.launch```

```$ rostopic pub /suav1/cam_target geometry_msgs/Point -r 1 -- '2.0' '0.0' '0.0'```

```$ rostopic pub /suav1/cmd_vel geometry_msgs/Twist -r 1 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.2]'```

## Contributors

Vengatesan Govindaraju

## License

See the [LICENSE](LICENSE.md) file for license rights and limitations (MIT).
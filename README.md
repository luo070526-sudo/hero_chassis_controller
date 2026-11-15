# hero_chassis_controller

ROS Noetic `ros_control` plugin for the four-wheel mecanum chassis in
[`rm_description_for_task`](https://github.com/Bluefairy-ljy/rm_description_for_task).

The controller subscribes to `/cmd_vel`, controls four wheel velocities with
independent `control_toolbox::Pid` loops, publishes `/odom`, and publishes the
`odom -> base_link` transform.

## Build

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build hero_chassis_controller
source devel/setup.bash
```

## Run

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch
```

Body-frame command:

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Global-frame command:

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch \
  command_frame:=odom
```

## PID

PID parameters are in `config/controllers.yaml`. The committed gains are
initial values and require tuning in Gazebo.

```bash
rosrun rqt_reconfigure rqt_reconfigure
rqt_plot /controller/hero_chassis_controller/pids/front_left/state/error
```

## Test

```bash
catkin build hero_chassis_controller
catkin run_tests hero_chassis_controller
catkin_test_results build/hero_chassis_controller
```

Use CLion from a terminal that has sourced ROS and the workspace. Attach the
debugger to `gzserver` to debug the loaded controller plugin.

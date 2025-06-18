# Introduction

This repository contains ROS interface packages for Unitree A1, Aliengo and Dogotix mini-cheetah, and MIT's Cheetah-Software modified to work with these robots. The `unitree_ros` module contains the robot desctiptions, lower-level control and Gazebo interface for the above three robots. The `quadruped_controller` contains the controller for locomotion. 

![A1_trotting_gait](https://user-images.githubusercontent.com/18585930/141070088-346f687b-6873-43ac-8912-bdecad1b574e.gif)

## Configuration 
Make sure the following exist in your ~/.bashrc file or export them into your terminal.

```
source /opt/ros/<YOUR-ROS-DISTRO>/setup.bash
soruce ~/<YOUR-CATKIN-WS-PATH>/devel/setup.bash
export GAZEBO_PLUGIN_PATH=~/<YOUR-CATKIN-WS-PATH>/devel/lib:${GAZEBO_PLUGIN_PATH}
```
## Build
To build
```
cd <YOUR-CATKIN-WS>
catkin_make 
```
## Running the code

To run the code

1. Launch Unitree A1 in gazebo simulation
```
roslaunch unitree_gazebo normal.launch rname:=robotname

```
where, robotname = `a1`, `aliengo`, `mini_cheetah`

2. Run the quadruped controller in a separate terminal
```
rosrun quadruped_controller mit_controller $ROBOT_ARG
```
where, $ROBOT_ARG = `a` for lauching Unitree A1, `A` for launching Aliengo and `m` for launching mini_cheetah

3. For controlling the robot via keyboard run,
```
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard

```

## Dependencies
* [ROS](https://www.ros.org/) melodic or ROS kinetic
* [Gazebo](http://gazebosim.org/)
* [Eigen](http://eigen.tuxfamily.org/)

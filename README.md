# ROSRagnarEDU
ROS interface for Ragnar EDU

## Dependencies
ROS Ragnar depends on the following core packages:
 1. [industrial_core](https://github.com/ros-industrial/industrial_core)
 2. rviz

**Please note that all terminals need to be sourced in your current workspace**

## Feedback and Visualization
To launch a node that will listen to the robot feedback and display a visualization in rviz:
```
roslaunch ragnar_drivers ragnar_state_listener.launch [robot_ip:=xxx.xxx.xxx.xxx]
```

Note that the ip for this node defaults to `192.168.1.240`

## Command Interface
To launch a node that enables commanding the robot to follow trajectories:
```
roslaunch ragnar_drivers ragnar_streaming_interface.launch robot_ip:=xxx.xxx.xxx.xxx
```

Note that the robot listens on the ```joint_path_command``` topic with type ```trajectory_msgs::JointTrajectory```

## Demonstrations
After these other components have been launched, you may run a test script with:
```
rosrun ragnar_kinematics ragnar_demo_motions 
```

**Please note that these motions may be unsafe**

## Outstanding Issues

This package is still in development, and requires a few components to be workable:

 1. The robot needs a ROS action server interface so that client programs can be aware of the state of
    robot motion
 2. The robot needs safety checks. There may be mechanical limits that dissallow certain joint configurations. If these are not caught by the kinematics routines or by a simple bounding box check on the work volume, they may damage the robot.
 3. The robot uses a custom state publisher (as opposed to the regular ROS one) and the Robot URDF needs to be adjusted to have ```floating``` joint types to enable plug-n-play ability with the existing system.

 There are more issues, but these are the big, oustanding ones. We welcome issues and pull-requests to make this software better.

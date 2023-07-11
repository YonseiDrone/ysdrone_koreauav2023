# 2023 koreauav Khadas VIM4 main-repo
**Autor: Jusuk Lee, ChanJoon Park, Inha Baek**

## offboard

This package provides the functionality for controlling the drone in offboard mode.

### Files

- [scripts](offboard/scripts): Directory containing the following scripts:
  - [control_node.py](offboard/scripts/control_node.py): Node for controlling the drone.
  - [path_node.py](offboard/scripts/path_node.py): Node for planning and following a path.
  - [setmode_node.py](offboard/scripts/setmode_node.py): Node for setting the flight mode of the drone.

## safety_landing

This package provides safety landing features for the drone.

### Files

- [scripts](safety_landing/scripts): Directory containing the following scripts:
  - [PID_control_node.py](safety_landing/scripts/PID_control_node.py): Node for PID control of the drone during landing.
  - [vision_kalman_filter_node.py](safety_landing/scripts/vision_kalman_filter_node.py): Node for filtering vision data using Kalman filter.

## ysdrone_msgs

This package defines custom ROS messages and services.

### Files

- [srv](ysdrone_msgs/srv): Directory containing the following service definition file:
  - [DroneCommand.srv](ysdrone_msgs/srv/DroneCommand.srv): Custom service for sending commands to the drone.

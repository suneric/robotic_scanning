# scan3d
**scan3d** is a robotic scanning system integrating with [Intel Realsense D435](https://www.intelrealsense.com/depth-camera-d435/) for 3D data acqusition, [KUKA LBR iiwa](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa) for performing 3D scanning, [ROS](https://www.ros.org/) for the communication between robotic arm and computer controller, [Point Cloud Library](http://pointclouds.org/) for 3D point cloud data processing and visualization. The system is developed in C++ on Ubuntu 16.04.

## Pre-requisites
- Hardware
    - Intel Realsense D435
    - KUKA LBR iiwa
- Software
    - Ubuntu 16.04
        - [ROS Kinetic](http://wiki.ros.org/kinetic)
        - [Intel Realsense SDK librealsense](https://github.com/IntelRealSense/librealsense)
        - [Point Cloud Library](http://pointclouds.org)
    - Micorsoft Windows 10
        -  KUKA Sunrise.Workbench

## Setup
KUKA Sunrise.Workbench will be running on a Windows machine, while other required software components will be running on Linux machine for leveraging ROS to communicate with KUKA LBR iiwa.   
1. Install KUKA Sunrise.Workench on Windows as KUKA LBR iiwa required.
2. Follow [IFL-CAMP/iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack/wiki) to [setup a Sunrise poject with iiwa_stack on SUNRISE Cabinet](https://github.com/IFL-CAMP/iiwa_stack/wiki/sunrise_project_setup).
3. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) and [Point Cloud Library](http://pointclouds.org/downloads/linux.html) on Ubuntu for C++.
4. Follow [IFL-CAMP/iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack/wiki) to [build and setup iiwa_stack on you ROS machine](https://github.com/IFL-CAMP/iiwa_stack/wiki/roscore_setup).

To verify your setup, you can start the program *ROSSmartServo* on KUKA Smartpad, and start
```
roscore
```
on your ROS machine. Start a new terminal to check ros topics with

```
rostopic list
```
all published topics will be listed, you could use

```
rostopic ehco /iiwa/state/JointPosition
```
to check if the Joint position of KUKA LBR iiwa could be queried, and use

```
rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition
```
with the message containing Joint position to check if a command could be sent to KUKA LBR iiwa to control the robot arm's position.

## Install
- Create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
- Clone the latest code from here into 'catkin_ws/src/'
```
git clone https://github.com/suneric/
cd ..
```
- Make the executable
```
catkin_make
```

## Features
Clone this repository and build with catkin_make, the executive "node" is located "/build/robotic_scan/".
Use the node for generating the trajecotry of camera with

```
rosrun robotic_scan scan3d -gt "../scan3d/trajectory/sample.txt"
```
and use the node for performing the scanning with a predefined trajectory

```
rosrun robotic_scan scan3d -scan "../scan3d/trajectory/sample.txt"
```
## Demo
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Q49_dNu2hSA/0.jpg)](https://www.youtube.com/watch?v=Q49_dNu2hSA)

## Developer
Yufeng Sun | sunyf@mail.uc.edu | IRAS Lab @ University of Cincinnati

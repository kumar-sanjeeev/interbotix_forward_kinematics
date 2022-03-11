# Simulating the forward Kinematics of Interbotix wx200 robotic Arm
This repository contains the python script of forward kinematics and ros package related files that I have used to simulate the forward kinematics of the serial manipulator arm in the RVIZ.

# System Setup

For solving the task ros package will be build with following dependencies. Use the following commands to install the required `dependencies`
```shell
$ git clone https://github.com/Interbotix/interbotix_ros_arms.git
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt update
$ sudo apt install ros-noetic-joint-state-publisher-gui
$ sudo apt install ros-noetic-controller-manager
$ sudo apt install ros-noetic-effort-controllers
$ sudo apt install ros-noetic-gazebo-ros-control
$ sudo apt install ros-noetic-joint-state-controller
$ sudo apt install ros-noetic-joint-trajectory-controller
$ sudo apt-get install ros-noetic-moveit
$ sudo apt-get install ros-noetic-moveit-visual-tools
$ sudo apt-get install ros-noetic-dynamixel-workbench-toolbox
```

### Task Description

- Compute the end effector position and orientation of the manipulator.
- Simulate the joint motion in the RVIZ
- Validate the joint angle and end effector position position with the conventional DH method

### Result:
https://user-images.githubusercontent.com/62834697/157987681-975a27b3-a52e-425c-9c07-c176431df6e9.mp4

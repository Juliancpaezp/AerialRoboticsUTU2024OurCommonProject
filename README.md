# Aerial Robotics UTU 2024: Group 5 - Common Project
 
## Description:
This code is meant to take off a trello drone, then move trough 
different check points using images from a on-board camera in a 
gazebo simulation, using ROS2 and OpenCV.

Based on: https://github.com/TIERS/drone_racing_ros2/tree/main

## Installation:
Run this on your console once: (or VisualStudio Code terminal, if you rather)

    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/TIERS/drone_racing_ros2.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build

## Usage
Run this every time you want to run the gazebo environment

    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo simple_launch.py

Then, run "OpenProjectAerialRobotics.py" with python from console

## Extra
You can also add the path to to your ".bashrc" file, if you want to run the python file faster...

    nano ~/.bashrc

And then add:

    # Drone_Racing_ROS2
    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    export GAZEBO_MODEL_PATH=/home/julian/drone_racing_ros2_ws/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh

Enjoy!
 
## Authors: 

     - Prashan Herath [prashan.r.herathmudiyanselage@utu.fi]
     - Julian C. Paez P. [julian.c.paezpineros@utu.fi]
     - Michalis Iona [michalis.l.iona@utu.fi]

For: University of Turku - TIERS 
Course: Aerial Robotics and Multi-Robot Systems 
Date: March 26th, 2024 

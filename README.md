# ROS transportation system using ROS1

Before running anything, first run catkin_make.

Run environment: roslaunch task_planner task_planner.launch

Run simulation: rosrun task_planner task_planner.py

## Description

This is a simple ROS simulation for a robot arm, and a robot car working together to transport a cube. Because of ROS moveit conflicts, only one arm was used which loads the cube onto the mobile robot, and unloading is done by despawning the cube.

To fix problems with older version of Gazebo not being able to grip and pick up a cube properly when using a robot arm, I used the grasp fix library from this repo: https://github.com/JenniferBuehler/gazebo-pkgs?tab=readme-ov-file
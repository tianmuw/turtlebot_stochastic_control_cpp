# turtlebot_stochastic_control_cpp
This is a stochastic controller program designed for the Turtlebot4 robot using C++ and Python in ROS2, Ubuntu 22.04. 
The program computes the optimal path for the robot given the goal position and static obstacle positions.

# Introduction
The Turtlebot4 robot is a popular platform used for research and education in robotics. 
This program provides a stochastic controller for the Turtlebot4, which can be used to 
compute the optimal path for the robot given the goal position and static obstacle positions. 
The program is implemented in C++ and Python, and uses ROS2 as the middleware.

# Installation
To install the program, follow these steps:
1. Clone the repository: 
```
cd ~/ros2_ws/src
git clone https://github.com/tianmuw/stochastic-controller.git
```
2. Install ROS2: follow the instructions on the ROS2 installation page 
(https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
3. Install any dependencies required by the program:
```
rosdep install --from-paths src --ignore-src -r -y
```
4. Build the program: 
```
cd ~/ros2_ws/
colcon build
```
# Algorithm Overview
The stochastic controller algorithm used in this program is based on the paper:
Time-Varying Safety Adaptation for Online Planning among Semantic Classes of Dynamic Environmental Actors
The algorithm computes the optimal path for the robot by sampling a set of random paths 
and selecting the path with the lowest cost. The cost function used in this program takes 
into account the distance to the goal position and the distance to the static obstacles.


# Micromouse-2.0

A maze-solving simulator designed to facilitate testing and development of heuristic-based algorithms for the [IEEE Micromouse Competition](https://ieeexplore.ieee.org/document/5971240). Built as part of an effort to improve my ROS skills and create a cool robotics application.

![image](https://github.com/user-attachments/assets/171bf9a4-60bf-46c8-b1b8-657414807692)
Robot sitting at start of maze.

## Features

- Randomly generated mazes built with recursive backtracking.
- Pre-built maze-solving algorithms (AStar, Flood Fill, etc).
- Differential drive robot model equipped with lidar for wall detection.
- Custom SLAM implementation to track robot's position and walls of maze.

## Installation

### Prerequisites
- ROS2 Humble
- C++
- Gazebo

### Setup
1. Clone the repository.
2. Build ROS2 workspace.
3. Compile C++ source code.

## Usage
1. Run 'maze_generation'.
2. Launch gazebo with 'simulation.launch.py'.
3. Run the navigation node to generate the maze binary, or use existing maze binary from maze_generation.
4. Compile and run the desired algorithm C++ program to generate the path binary.
5. Relaunch the environment and run the traversal node to follow the shortest path to the goal.

https://github.com/user-attachments/assets/2f705e0a-298e-47a5-b865-a64ee07f9ce2

Robot navigating the maze using floodfill-generated path.

## What I Learned
1. First time building my own project with ROS! I learned a lot about configuring the simulated environment, as well as different control issues that can arise when moving around in a simulated environment.
2. Had to explore a few cool algorithms for maze generation and solving.
3. Learned to implement state-based control of my simulated robot, as well as the nuances of publishing and subscribing.
4. How to use lidar for object detection in a simulated environment.


## Acknowledgement
- Learned a lot from this playlist by [Articulated Robotics](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT).
- [ROS Humble documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate.html) was very helpful as well.

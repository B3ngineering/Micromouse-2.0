# Micromouse-2.0

A maze-solving simulator designed to facilitate testing and development of heuristic-based algorithms for the [IEEE Micromouse Competition](https://ieeexplore.ieee.org/document/5971240). Built as part of an effort to improve my ROS skills and create a cool robotics application.

## Features

- Randomly generated mazes built with recursive backtracking
- Pre-built maze-solving algorithms (AStar, Flood Fill, etc)
- Differential drive robot model equipped with Lidar for wall detection

## Installation

### Prerequisites
- ROS2 Humble
- C++
- Gazebo

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/B3ngineering/Micromouse-V2.git
   cd ros2_ws
   ```
2. Build ROS2 workspace
3. Compile C++ source code

## Usage
1. Run maze generation
2. Select the algorithm to use and run it's C++ generation code
3. Build the ROS2 workspace again
4. Run the gazebo environment


## Acknowledgement
= Learned a lot from this playlist by [Articulated Robotics](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)

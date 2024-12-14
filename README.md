# TurtleBot3 Maze Project

This project uses Q-learning to navigate a TurtleBot3 through a maze to reach a goal position.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Setup Instructions](#setup-instructions)
3. [Running the Code](#running-the-code)
4. [Folder Structure](#folder-structure)
5. [Results](#results)

---

## Project Overview

This project focuses on using a Q-learning algorithm to autonomously navigate a TurtleBot3 robot through a maze in a Gazebo simulation environment.

- **Robot**: TurtleBot3
- **Goal**: Reach the ball in the top-right corner of the maze.
- **Framework**: ROS 2 with Gazebo

---

## Setup Instructions

### Prerequisites
1. Install **ROS 2 Humble**:
   - Follow the installation guide: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html).
2. Install TurtleBot3 packages:
   ```bash
   sudo apt install ros-humble-turtlebot3*
3. pip install -r requirements.txt

### Setting UP the Workspace
1. Clone this repository
  git clone https://github.com/johnsonkim2000/turtlebot3_maze_runner
  cd turtlebot3_maze_project
2. Source ROS 2 Setup Files
  source /opt/ros/humble/setup.bash
  source ~/turtlebot3_ws/install/setup.bash

## Running the Code

### Launch the Maze Environment
1. Open a terminal and run:
   ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/turtlebot3_maze_project/worlds/Final.world

### Start the Q-learning Node
2. In a new terminal:
   source /opt/ros/humble/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   ros2 run q_learning_maze_runner q_learning_maze_runner

## Folder Structure

turtlebot3_maze_project/
├── README.md
├── scripts/
│   ├── q_learning_maze_runner.py
│   ├── lidar.py
├── worlds/
│   ├── Final.world
├── logs/
│   ├── q_table.csv
│   ├── rewards.csv
├── requirements.txt
├── setup.bash
└── LICENSE


## Results
1. Q-table: Saved in logs/q_table.csv
2. Rewards: Recorded in logs/rewards.csv
3. The robot learns over multiple iterations to navigate through the maze




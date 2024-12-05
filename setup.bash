 Source ROS 2
source /opt/ros/humble/setup.bash

# Source TurtleBot3 workspace
source ~/turtlebot3_ws/install/setup.bash

# Set Gazebo paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/johnson/turtlebot3_maze_project/worlds

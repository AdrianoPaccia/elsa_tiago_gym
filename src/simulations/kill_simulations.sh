#!/bin/bash

# Terminate all processes with SIGTERM
#pkill -TERM -P $$

# Terminate all processes with specific names or patterns
pkill -f "gazebo_simulation.sh $"
pkill -f "roslaunch tiago_gazebo"

# Optionally, wait for processes to terminate gracefully
sleep 2

# Confirm the termination of processes
pkill -0 -f "gazebo_simulation.sh $" && echo "Failed to terminate gazebo_simulation.sh processes"
pkill -0 -f "roslaunch tiago_gazebo" && echo "Failed to terminate roslaunch tiago_gazebo processes"
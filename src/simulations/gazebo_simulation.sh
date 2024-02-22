#!/bin/bash

# Check if the user provided the index i as a command-line argument
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <index_i>"
    exit 1
fi

# Extract the index i from the command-line argument
index_i="$1"

# Check if the provided argument is a valid number
if ! [[ "$index_i" =~ ^[0-9]+$ ]]; then
    echo "Error: Please provide a valid index i."
    exit 1
fi

# Set ROS_MASTER_URI based on the index i
export ROS_MASTER_URI=http://localhost:1135$index_i
export GAZEBO_MASTER_URI=http://localhost:1134$index_i

# Redirect output to a log file
logfile="simulation_$index_i.log"
exec > >(tee -i "$logfile")
exec 2>&1

# Add your specific commands here
echo "Simulation client #$index_i..."
roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=true tuck_arm:=false 

# Create a flag file to indicate that the simulation is done
#echo "Simulation done" > "simulation_done_$index_i.flag"

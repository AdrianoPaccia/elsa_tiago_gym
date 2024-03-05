#!/bin/bash

# Check if the user provided the index i as a command-line argument
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <index_i> <sim_velocity> <gui>"
    exit 1
fi

# Extract the index i and the velocity from the command-line argument
index_i="$1"
velocity="$2"
gui="$3"

# Check if the provided argument is a valid number
if ! [[ "$index_i" =~ ^[0-9]+$ ]]; then
    echo "Error: Please provide a valid index i."
    exit 1
fi

# Set ROS_MASTER_URI based on the index i
export ROS_MASTER_URI=http://localhost:1135$index_i
export GAZEBO_MASTER_URI=http://localhost:1134$index_i

# Redirect output to a log file
#logfile="simulation_$index_i.log"
#exec > >(tee -i "$logfile")
#exec 2>&1

#get the relative path
script_dir="$(cd "$(dirname $0)" && pwd)"
current_dir="$(pwd)"
relative_path=$(realpath --relative-to="$current_dir" "$script_dir")

# Add your specific commands here
roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=$gui tuck_arm:=false 
sleep 2

# Create a flag file to indicate that the simulation is done
#echo "Simulation done" > "simulation_done_$index_i.flag"

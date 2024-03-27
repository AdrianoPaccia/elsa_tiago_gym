#!/bin/bash

# Check if the user provided the required arguments
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <sim_velocity> <gui>"
    exit 1
fi

# Extract the arguments
velocity="$1"
gui="$2"
echo "velocity = $velocity"
echo "gui = $gui"

# Get the relative path
script_dir="$(cd "$(dirname "$0")" && pwd)"

roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=$gui tuck_arm:=false > "$script_dir/logs/output_master.log" 2>&1 &
pid=$!
pids+=("$pid")
echo "Master simulation (gui=$gui) is up and running with PID: $pid"

# Wait for all background processes to finish
wait "$pid"


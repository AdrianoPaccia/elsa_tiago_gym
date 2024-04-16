#!/bin/bash

# Check if the user provided the required arguments
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <gui>"
    exit 1
fi

# Extract the arguments
gui="$1"
echo "gui = $gui"

# Get the relative path
#script_dir="$(cd "$(dirname "$0")" && pwd)"
log_dir="$(pwd)/save/ros_logs"
mkdir -p "$log_dir"

if ! rostopic list &>/dev/null && ! rosnode list &>/dev/null; then
    echo "Launching simulation..."
    roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=$gui tuck_arm:=false > "$log_dir/output_master.log" 2>&1 &
    pid=$!
    pids+=("$pid")
    echo "Master simulation (gui=$gui) is up and running with PID: $pid"

    # Wait for all background processes to finish
    wait "$pid"
else
    echo "Simulation is already running"
    exit 1
fi

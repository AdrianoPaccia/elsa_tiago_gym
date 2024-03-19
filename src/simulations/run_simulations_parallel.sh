#!/bin/bash

# Check if the user provided the required arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <number_of_scripts>  <sim_velocity> <gui>"
    exit 1
fi

# Extract the arguments
num_scripts="$1"
velocity="$2"
gui="$3"

# Check if the provided argument is a valid number
if ! [[ "$num_scripts" =~ ^[0-9]+$ ]]; then
    echo "Error: Please provide a valid number of scripts."
    exit 1
fi

# Store PIDs of background processes
pids=()

# Get the relative path
script_dir="$(cd "$(dirname "$0")" && pwd)"

# Open tabs for each script in a new window
for ((i=0; i<num_scripts; i++)); do
    (
    export ROS_MASTER_URI="http://localhost:1135$i"
    export GAZEBO_MASTER_URI="http://localhost:1134$i"
    roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=$gui tuck_arm:=false > "$script_dir/logs/output_worker$i.log" 2>&1 &
    pid=$!
    pids+=("$pid")
    echo "Simulation $i is up and running with PID: $pid"
    sleep 5
    gz physics -u 0 -s "$velocity"
    ) &
done

roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=true tuck_arm:=false > "$script_dir/logs/output_master.log" 2>&1 &
master_pid=$!
pids+=("$master_pid")
echo "Master simulation is up and running with PID: $master_pid"

echo "All simulations are launching..."

# Wait for all background processes to finish
for pid in "${pids[@]}"; do
    wait "$pid"
    echo "Simulation with PID $pid has finished"
done

echo "All simulations have finished"

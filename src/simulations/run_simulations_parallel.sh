#!/bin/bash

# Check if the user provided a number as a command-line argument
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <number_of_scripts>  <sim_velocity> <gui>"
    exit 1
fi

# Extract the number of scripts from the command-line argument
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

#get the relative path
script_dir="$(cd "$(dirname $0)" && pwd)"
current_dir="$(pwd)"
relative_path=$(realpath --relative-to="$current_dir" "$script_dir")

# Open tabs for each script in the new window
for ((i=0; i<$num_scripts; i++)); do
    (
    export ROS_MASTER_URI="http://localhost:1135$i"
    export GAZEBO_MASTER_URI="http://localhost:1134$i"
    roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=$gui tuck_arm:=false > "$script_dir/logs/output_worker$i.log" 2>&1
    sleep 5
    gz physics -u 0 -s $velocity
    
    ) &
    pids+=($!) # Store the PID of the last background process    
done

roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true gui:=false tuck_arm:=false > "$script_dir/logs/output_master.log" 2>&1 &
pids+=($!) # Store the PID of the last background process

# Wait for all background processes to finish
for pid in "${pids[@]}"; do
    wait "$pid"
done

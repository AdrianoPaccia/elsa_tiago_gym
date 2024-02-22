#!/bin/bash

# Check if the user provided a number as a command-line argument
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <number_of_scripts>  <sim_velocity>"
    exit 1
fi

# Extract the number of scripts from the command-line argument
num_scripts="$1"
velocity="$2"

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
# Run scripts for the worker simulator environments
for ((i=0; i<$num_scripts; i++)); do
    gnome-terminal -- bash -c "./$relative_path/gazebo_simulation.sh $i; exec bash" &

    #"./$relative_path/set_velocity.sh" "$velocity" &
    pids+=($!)  # Store the PID of the last background process
    sleep 10
done

# Run scripts for the principal simulator environment
gnome-terminal -- bash -c "roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true; exec bash"

#roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true &
pids+=($!)

# Wait for all background processes to finish
for pid in "${pids[@]}"; do
    wait "$pid"
done
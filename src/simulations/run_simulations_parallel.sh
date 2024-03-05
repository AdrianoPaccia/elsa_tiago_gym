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

#silence the WARN and INFO ros logs
export ROS_LOG_DIR=error 

# Store PIDs of background processes
pids=()

#get the relative path
script_dir="$(cd "$(dirname $0)" && pwd)"
current_dir="$(pwd)"
relative_path=$(realpath --relative-to="$current_dir" "$script_dir")

# Create a new terminal window

# Open tabs for each script in the new window
for ((i=0; i<$num_scripts; i++)); do
    gnome-terminal --tab --title="Simulation #$i" -- bash -c "
    ./$relative_path/gazebo_simulation.sh $i $velocity $gui;
    exec bash;
    rosparam set /rosout/level fatal;
    exec bash;" &
    sleep 5
done
# Run scripts for the worker simulator environments
#for ((i=0; i<$num_scripts; i++)); do
#    gnome-terminal -- bash -c "./$relative_path/gazebo_simulation.sh $i $velocity $gui; exec bash" &
#    pids+=($!)  # Store the PID of the last background process
#    sleep 10
#done

# Run scripts for the principal simulator environment
gnome-terminal --tab -- bash -c "
    roslaunch tiago_gazebo tiago_gazebo.launch world:=elsa end_effector:=robotiq-2f-85 public_sim:=true tuck_arm:=false gui:=false;
    rosparam set /rosout/level fatal;,
    ./$relative_path/set_velocity.sh $velocity;
     exec bash" &
pids+=($!) # Store the PID of the last background process
sleep 10

# Wait for all background processes to finish
for pid in "${pids[@]}"; do
    wait "$pid"
done

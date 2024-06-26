#!/bin/bash

# Check if the user provided the required arguments
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <number_of_scripts> <gui>"
    exit 1
fi

# Extract the arguments
num_scripts="$1"
gui="$2"

# Check if the provided argument is a valid number
if ! [[ "$num_scripts" =~ ^[0-9]+$ ]]; then
    echo "Error: Please provide a valid number of scripts."
    exit 1
fi

# Store PIDs of background processes
pids=()

# Get the relative path
#script_dir="$(cd "$(dirname "$0")" && pwd)"
log_dir="$(pwd)/save/ros_logs"
mkdir -p "$log_dir"

world=elsa
gui_master=$gui



export ROS_MASTER_URI="http://localhost:11311"
export GAZEBO_MASTER_URI="http://localhost:11312"
if ! rostopic list &>/dev/null && ! rosnode list &>/dev/null; then
    (echo "Launching Master simulation..."
    roslaunch tiago_gazebo tiago_gazebo.launch world:=$world end_effector:=robotiq-2f-85 public_sim:=true gui:=$gui_master tuck_arm:=false > "$log_dir/output_master.log" 2>&1) &
    pid=$!
    pids+=("$pid")
    echo "Master simulation (gui=$gui_master) is up and running with PID: $pid"

else
    echo "Master Simulation is already running"
    sleep 5

fi


# Open tabs for each script in a new window
#elsa_with_tiago_omni base_type:=omni_base
for ((i=0; i<num_scripts; i++)); do
    (
    export ROS_MASTER_URI="http://localhost:1135$i"
    export GAZEBO_MASTER_URI="http://localhost:1134$i"
    if ! rostopic list &>/dev/null && ! rosnode list &>/dev/null; then
        echo "Launching simulation $i"
        roslaunch tiago_gazebo tiago_gazebo.launch world:=$world end_effector:=robotiq-2f-85 public_sim:=true gui:=$gui tuck_arm:=false > "$log_dir/output_worker$i.log" 2>&1 &
        pid=$!
        pids+=("$pid")
        echo "Worker simulation $i (gui=$gui) is up and running with PID: $pid"
        sleep 2
    else
        echo "Simulation $i is already running"
        sleep 5
    fi
    ) &
done

# Wait for all background processes to finish
for pid in "${pids[@]}"; do
    wait "$pid"
    echo "Simulation with PID $pid has finished"
done

echo "All simulations have finished"
exit 0
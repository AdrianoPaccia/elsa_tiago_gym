#!/bin/bash

# Check if the user provided the velocity as a command-line argument
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <n> <velocity>"
    exit 1
fi

# Extract the velocity from the command-line argument
N=$1
velocity=$2

for ((i = 0; i < $N; i++)); do
    export GAZEBO_MASTER_URI=http://localhost:1134$i
    gz physics -u 0 -s $velocity
done

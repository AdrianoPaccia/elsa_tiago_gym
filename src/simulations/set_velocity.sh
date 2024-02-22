#!/bin/bash

# Check if the user provided the velocity as a command-line argument
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <velocity>"
    exit 1
fi

# Extract the velocity from the command-line argument
velocity="$1"

# Set the physics simulation step size to the provided velocity
gz physics –u 0 -s "$velocity"
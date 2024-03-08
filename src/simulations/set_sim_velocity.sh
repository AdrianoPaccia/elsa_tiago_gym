#!/bin/bash

# Check if the user provided the velocity as a command-line argument
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <gz_master_uri> <velocity> "
    exit 1
fi

# Extract the velocity from the command-line argument
gz_master_uri=$1
velocity=$2

(export GAZEBO_MASTER_URI=http://localhost:1134$gz_master_uri
gz physics -u 0 -s $velocity)&
 
#gnome-terminal --geometry=80x24+0+0 -- bash -c "
#    echo 'GAZEBO_MASTER_URI=$gz_master_uri'
#    export GAZEBO_MASTER_URI=$gz_master_uri;
#    gz physics -u 0 -s $velocity;
#    exit"



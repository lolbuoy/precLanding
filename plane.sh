#!/bin/bash

# Specify the directory where your Python script is located
echo "Plane Running"
target_directory="/home/abhinav/Desktop/ardupilot/Tools/autotest"

# Change to the specified directory
cd "$target_directory" || exit 1

# Run your Python script
python sim_vehicle.py -v Plane --map --console

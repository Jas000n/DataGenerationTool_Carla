#!/bin/bash

count=0
while [ $count -lt 100 ]
do
    echo "Attempting run number $((count+1))..."
    python sub_process.py
    result=$?
    echo "return value $result"
    if [ $result -eq 0 ]; then
        echo "Run number $((count+1)) succeeded."
        ((count++))
    elif [ $result -eq 139 ]; then  # Usually, 139 indicates a segmentation fault (signal 11)
        echo "Segmentation fault detected, handling..."
        # Find and kill Carla process
        pkill -f CarlaUE4-Linux-
        pkill -f CarlaUE4-Linux-
        # Wait some time to let the process fully stop
        sleep 10
        # Restart Carla
        ./carla/CarlaUE4.sh &
        sleep 20  # Wait for Carla to start
        echo "Carla restarted, retrying..."
    elif [ $result -eq 100 ]; then  # no carla server found, start carla
        pkill -f CarlaUE4-Linux-
        pkill -f CarlaUE4-Linux-
        # Wait some time to let the process fully stop
        sleep 10
        # Restart Carla
        ./carla/CarlaUE4.sh &
        sleep 20  # Wait for Carla to start
        echo "Carla restarted, retrying..."
    else
        echo "Run number $((count+1)) failed, error code $result, retrying..."
    fi
done

echo "All 200 runs completed successfully."







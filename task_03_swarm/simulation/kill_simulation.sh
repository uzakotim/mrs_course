#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./singularity.sh exec "source ~/.bashrc && ( tmux kill-session -t boids || echo no boids running ) && ( tmux kill-session -t simulation || echo no simulation running ) && ( killall mavros_node mavros px4 gazebo_ros gzserver gzclient gazebo roscore rosout rosmaster || echo done)"
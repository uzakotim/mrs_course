#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./singularity.sh exec "source ~/.bashrc && cd ~/task_02_formation/simulation/user_ros_workspace/src/formation && code ./ src/formation.cpp include/student_headers/formation.h include/student_headers/astar.h config/reshaping_debug.yaml"

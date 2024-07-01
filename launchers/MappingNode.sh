#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package MappingNode.py & 
rosrun my_package odometrynode.py

# wait for app to end
dt-launchfile-join

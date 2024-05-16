#!/bin/bash

source /environment.sh

dt-exec echo "launching my-publisher node" 
# initialize launch file
dt-launchfile-init

# launch publisher
rosrun my_package my_publisher_node.py

# wait for app to end
dt-launchfile-join

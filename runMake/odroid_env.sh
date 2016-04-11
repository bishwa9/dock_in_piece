#!/bin/bash

source ~/.bashrc
export ROS_MASTER_URI=http://10.1.1.1:11311
export ROS_IP=10.1.1.2
export ROS_HOSTNAME=10.1.1.2
export DISPLAY=:0.0
export ROS_WS=/home/odroid/Desktop/dji_code
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS

exec "$@"
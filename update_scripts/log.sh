#!/bin/bash 	

source dji_code/devel/setup.bash
rosbag record -o ./log/ -a -x "/usb_cam/(.*)"
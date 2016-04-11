#!/bin/bash

sudo swapon /swapfile
catkin_make
sudo swapoff /swapfile
bash
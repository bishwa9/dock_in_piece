#!/bin/bash

#Author: Bishwamoy Sinha Roy
#Updates the client package on the odroid by secure copying the contents of the client_pkg

./update_djiSDK.sh
./update_guidance.sh
./update_client.sh
./update_aprilTag.sh
./update_comm.sh
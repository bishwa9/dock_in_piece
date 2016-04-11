#!/bin/bash

#Author: Bishwamoy Sinha Roy
#Updates the client package on the odroid by secure copying the contents of the client_pkg

scp -r ./dji_code/src/navigation_pkg/* odroid@10.1.1.2:~/Desktop/dji_code/src/navigation_pkg/
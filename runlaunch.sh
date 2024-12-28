#!/bin/bash

#xhost +
cd ..
. install/setup.bash

ros2 launch bcn3d_moveo_moveit bcn3d_moveo.launch.py

sleep 12

ros2 run bcn3d_moveo_scripts go_go_generate

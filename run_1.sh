#!/bin/bash

xhost +local:

sudo docker run --name bcn3d_moveo_1 -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=31 \
    bcn3d_moveo /bin/bash 
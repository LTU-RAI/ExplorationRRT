#!/bin/bash

XAUTH=/path/to/Xauthority
# Allow local Docker containers to access the X server
xhost +local:docker
docker run -it --rm \
    --name errt_test \
    --gpus all \
    --runtime=nvidia \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES" \
    $DOCKER_VISUAL_NVIDIA \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    # -v /home/aakapatel/catkin_workspaces:/home/aakapatel/catkin_workspaces \
    # -v /home/aakapatel/.gazebo/models:/home/aakpatel/.gazebo/models \
    errt_test

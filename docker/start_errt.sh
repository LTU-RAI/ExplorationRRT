#!/bin/bash

docker run -it --rm --name stage --gpus all --runtime=nvidia \                                                                                                                                 took 16s
    --net=host \
    -e DISPLAY=$DISPLAY \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES" \
    $DOCKER_VISUAL_NVIDIA \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    -v /home/aakapatel/catkin_workspaces:/home/aakapatel/catkin_workspaces \
    -v /home/aakapatel/.gazebo/models:/home/aakpatel/.gazebo/models \
    errt

#!/bin/bash

XAUTH=/path/to/Xauthority
# Allow local Docker containers to access the X server
xhost +local:docker
docker run -it --rm \
    --name errt_test \
    --gpus all \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    errt_test

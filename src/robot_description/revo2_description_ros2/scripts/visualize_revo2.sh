#!/bin/bash

# Check if Docker image exists, build if not
if ! docker image inspect revo2_description_ros2 >/dev/null 2>&1; then
    echo "Building revo2_description_ros2 Docker image..."
    docker build -t revo2_description_ros2 .docker
    echo "Docker image built successfully."
else
    echo "Using existing revo2_description_ros2 Docker image."
fi

echo

# docker run -it -u $(id -u) \
#     --privileged \
#     -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
#     -v $(pwd):/workspaces/src/revo2_description \
#     -w /workspaces/src/revo2_description \
#     revo2_description_ros2 \
#     .docker/visualize_revo2.entrypoint.sh $*

docker run -it --rm \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=${DISPLAY} \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v $(pwd):/workspaces/src/revo2_description \
    -w /workspaces \
    revo2_description_ros2 \
    /workspaces/src/revo2_description/.docker/visualize_revo2.entrypoint.sh $*

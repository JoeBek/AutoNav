#!/bin/bash

set -e # makes script exit on command failure

# set params

IMAGE_ID="autonav:koopa-kingdom"
CONTAINER_NAME="koopa-kingdom"
WORKDIR="$HOME/AutoNav/isaac_ros-dev/"
ENTRYPOINT="$HOME/AutoNav/env/docker/entrypoint.sh"


DOCKER_ARGS=()

# networking
DOCKER_ARGS+=("--network host") # host network stack
DOCKER_ARGS+=("--ipc=host") # shares IPC namespace

# GPU
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

# display forwarding
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix") # mounts socket
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw") # mounts xauth file 
DOCKER_ARGS+=("-e DISPLAY") # set display env var


# jetson specific stuff
if [[ $PLATFORM == "aarch64" ]]; then

	DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats") # mount tegra stats
	DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra") # mount tegra stats
	DOCKER_ARGS+=("--pid-host")

# mount dev fs, entrypoint, set working directory
DOCKER_ARGS+=("-v $WORKDIR:/workspace/isaac_ros-dev") # mount workspace 
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro") # sync time or something
DOCKER_ARGS+=("--workdir /workspace/isaac_ros-dev") # mount workspace 
#DOCKER_ARGS+=("--entrypoint $ENTRYPOINT")


docker run -it --rm \
	--privileged \
	${DOCKER_ARGS[@]} \
	--name "$CONTAINER_NAME" \
	--runtime nvidia \
	$IMAGE_TAG \
	/bin/bash







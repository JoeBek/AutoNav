#!/bin/bash

set -e # makes script exit on command failure

# set params

IMAGE_TAG="dev:koopa-kingdom"
CONTAINER_NAME="koopa-kingdom"
WORKDIR="$HOME/AutoNav/isaac_ros-dev/"
ENTRYPOINT="/usr/local/bin/scripts/entrypoint.sh"
SCRIPT_DIR="$(dirname ${BASH_SOURCE[0]})"



DOCKER_ARGS=()

# networking
#DOCKER_ARGS+=("--network host") # host network stack
#DOCKER_ARGS+=("--ipc=host") # shares IPC namespace
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")
DOCKER_ARGS+=("-e HOST_USER_UID=`id -u`")
DOCKER_ARGS+=("-e HOST_USER_GID=`id -g`")
DOCKER_ARGS+=("-e WORKDIR=$WORKDIR")



# GPU
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

# display forwarding
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix") # mounts socket
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw") # mounts xauth file 
DOCKER_ARGS+=("-e DISPLAY") # set display env var

# forward SSH agent
if [[ -n $SSH_AUTH_SOCK ]]; then
    DOCKER_ARGS+=("-v $SSH_AUTH_SOCK:/ssh-agent")
    DOCKER_ARGS+=("-e SSH_AUTH_SOCK=/ssh-agent")
fi

# jetson specific stuff
if [[ $PLATFORM == "aarch64" ]]; then
    DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=nvidia.com/gpu=all,nvidia.com/pva=all")
       DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/:/tmp/")
       DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
       DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v /usr/share/vpi3:/usr/share/vpi3")
       DOCKER_ARGS+=("-v /dev/input:/dev/input")

       if [[ $(getent group jtop) ]]; then
	       DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
       fi
fi


# mount dev fs, entrypoint, set working directory
DOCKER_ARGS+=("-v $WORKDIR:/workspace/isaac_ros-dev") # mount workspace 
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro") # sync time or something
DOCKER_ARGS+=("--workdir /workspace/isaac_ros-dev") # mount workspace 
DOCKER_ARGS+=("-v $SCRIPT_DIR/entrypoint_additions:/usr/local/bin/scripts/entrypoint_additions") # mount entrypoint scripts
DOCKER_ARGS+=("-v $SCRIPT_DIR/entrypoint.sh:/usr/local/bin/scripts/entrypoint.sh") # mount entrypoint
#DOCKER_ARGS+=("-e PS1='bowser@koopa-kingdom:\\w # '") # set cool prompt
DOCKER_ARGS+=("--entrypoint $ENTRYPOINT")




# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    C_WORKDIR=$(docker exec $CONTAINER_NAME printenv WORKDIR)
    print_info "Docker workspace: $C_WORKDIR"
    docker exec -i -t -u admin --workdir $C_WORKDIR $CONTAINER_NAME /bin/bash $@
    exit 0
fi



docker run -it --rm \
	--privileged \
	--network host \
	--ipc=host \
	${DOCKER_ARGS[@]} \
	--name "$CONTAINER_NAME" \
	--runtime nvidia \
	$IMAGE_TAG \
	/bin/bash


#!/bin/bash
SCRIPT_DIR="$(dirname $(readlink -f $0))"
REPO_DIR="$(realpath "${SCRIPT_DIR}/..")"
PARENT_DIR="$(realpath "${SCRIPT_DIR}/../..")"
GRANDPARENT_DIR="$(realpath "${SCRIPT_DIR}/../../..")"

xhost +
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host --pid=host --ipc=host --privileged \
    -e "PRIVACY_CONSENT=Y" \
	-v /dev:/dev \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -v $REPO_DIR:/thesis:rw \
    nvcr.io/nvidia/isaac-sim:4.5.0 \
    /thesis/bimanual_teleoperation/entrypoint_isaac_sim.sh 
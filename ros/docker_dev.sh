#!/bin/bash

# Script to create and open ROS Noetic development container
# Uses the existing Dockerfile.ros1 and mounts current repository for building

CONTAINER_NAME="ros-dev-noetic"
IMAGE_NAME="cepton-ros1-dev"
REPO_PATH="$(pwd)"
SDK_PATH="$(pwd)/../cepton-sdk-3.0.20.5"
DOCKERFILE_PATH="../ci/Dockerfile.ros1"

# Build the Docker image if it doesn't exist
if ! docker image inspect ${IMAGE_NAME} >/dev/null 2>&1; then
    echo "Building Docker image: ${IMAGE_NAME}"
    docker build -f ${DOCKERFILE_PATH} -t ${IMAGE_NAME} ../ci/
fi

# Stop and remove existing container if it exists
if docker ps -a --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Stopping and removing existing container: ${CONTAINER_NAME}"
    docker stop ${CONTAINER_NAME} 2>/dev/null
    docker rm ${CONTAINER_NAME} 2>/dev/null
fi

# Create and run new container
echo "Creating new ROS Noetic container: ${CONTAINER_NAME}"
echo "Mounting repository: ${REPO_PATH} -> /catkin_ws/src/cepton_ros"
echo "Mounting SDK: ${SDK_PATH} -> /catkin_ws/src/cepton-sdk-3.0.20.5"

docker run -it \
    --name ${CONTAINER_NAME} \
    --mount type=bind,source="${REPO_PATH}",target=/catkin_ws/src/cepton_ros \
    --mount type=bind,source="${SDK_PATH}",target=/catkin_ws/src/cepton-sdk-3.0.20.5 \
    --workdir /catkin_ws \
    ${IMAGE_NAME} \
    /bin/bash

echo "Container ${CONTAINER_NAME} has exited"
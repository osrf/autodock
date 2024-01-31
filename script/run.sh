#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config.sh

xhost +local:root 
docker container prune -f 

rocker --x11 \
    --nvidia \
    --name=${CONTAINER_NAME} \
    --network=${ROS_NETWORK} \
    --env TURTLEBOT3_MODEL=burger \
    -- ${DOCKER_IMAGE_NAME} roscore

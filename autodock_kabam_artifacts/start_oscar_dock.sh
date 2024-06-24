#!/bin/bash

# clear

# docker build -t ros:autodock -f Dockerfile_OscarDeploy .

docker stop oscar_dock

docker rm oscar_dock

docker run -it \
--network=host \
--env-file runtime.env \
-v $HOME/.cognicept/oscar_dock_config.yaml:/root/catkin_ws/src/autodock/autodock_kabam/configs/oscar.yaml \
--name=oscar_dock  \
--restart=unless-stopped \
412284733352.dkr.ecr.ap-southeast-1.amazonaws.com/ros:autodockv2

# --env-file runtime.env \
# -v $HOME/.cognicept/oscar_dock_config.yaml:/root/catkin_ws/src/autodock/autodock_kabam/configs/oscar.yaml \
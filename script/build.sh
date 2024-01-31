#!/bin/bash

set -e

CURRENT_DIR="$(pwd)"

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config.sh

cd "$PROJECT_ROOT"
docker build -t ${DOCKER_IMAGE_NAME} .

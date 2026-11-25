#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd ${MY_PATH}

## --------------------------------------------------------------
## |                            setup                           |
## --------------------------------------------------------------

source ./common_vars.sh

## --------------------------------------------------------------
## |                            build                           |
## --------------------------------------------------------------

# px4_msgs must match the message definitions of the base image's PX4 (versioned topics): clone
# px4_msgs / px4_ros_com once and take the .msg / .srv files from $PX4_DIR in the image
# (rm -rf src/px4_msgs src/px4_ros_com to refetch, e.g. after the base image got a new PX4)
if [ ! -d src/px4_msgs ]; then
  git clone -q --depth 1 https://github.com/PX4/px4_msgs.git src/px4_msgs
  rm -f src/px4_msgs/msg/*.msg src/px4_msgs/srv/*.srv
  docker run --rm --entrypoint bash ${BASE_IMAGE} -c \
    'cd $PX4_DIR && tar -c msg/*.msg $(ls msg/versioned/*.msg 2>/dev/null) srv/*.srv' | tar -x -C src/px4_msgs
  [ -d src/px4_msgs/msg/versioned ] && mv src/px4_msgs/msg/versioned/*.msg src/px4_msgs/msg/ && rmdir src/px4_msgs/msg/versioned
fi
[ -d src/px4_ros_com ] || git clone -q --depth 1 https://github.com/PX4/px4_ros_com.git src/px4_ros_com

# initialize the cache
[ ! -e ${CACHE_PATH}/${WORKSPACE_PATH} ] && mkdir -p ./${CACHE_PATH}/${WORKSPACE_PATH}

PASS_TO_DOCKER_BUILD="Dockerfile src ${CACHE_PATH}/${WORKSPACE_PATH} colcon_defaults.yaml"

docker buildx use default

echo ""
echo "$0: building the user's workspace for $ARCH"
echo ""

# this first build compiles the contents of "src" and storest the intermediate
tar -czh $PASS_TO_DOCKER_BUILD 2>/dev/null | docker build - --pull=false --no-cache --target stage_cache_workspace --output ./${CACHE_PATH} --build-arg WORKSPACE_PATH=${WORKSPACE_PATH} --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg TRANSPORT_IMAGE=${TRANSPORT_IMAGE} --build-arg ARCH=${ARCH} --file Dockerfile --platform=linux/$ARCH

echo ""
echo "$0: packing the workspace into a docker image"
echo ""

# this second build takes the resulting workspace and storest in in a final image
# that can be deployed to a drone
docker build . --no-cache --target stage_finalization --file Dockerfile --build-arg WORKSPACE_PATH=${WORKSPACE_PATH} --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg TRANSPORT_IMAGE=${TRANSPORT_IMAGE} --build-arg ARCH=${ARCH} --tag $OUTPUT_IMAGE --platform=linux/$ARCH

echo ""
echo "$0: workspace was packed into '$OUTPUT_IMAGE'"
echo ""

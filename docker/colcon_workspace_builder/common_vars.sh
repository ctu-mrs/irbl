# base image: smart-deployment/docker/lazydocker/base_image/build.sh
export BASE_IMAGE=smart-deployment:latest

export TRANSPORT_IMAGE=alpine:latest

# the workspace packed into an image (./export_image.sh), not needed by ../sim
export OUTPUT_IMAGE=irbl_workspace:latest

export EXPORT_PATH=~/docker

export ARCH=amd64

# where the workspace lives in the image / the colcon_workspace volume (the base image's entrypoint sources it)
export WORKSPACE_PATH=etc/docker/colcon_workspace

export CACHE_PATH=cache

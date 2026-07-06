#!/bin/bash

set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <src_rosbag> <dst_rosbag>"
  exit 1
fi

SRC="$1"
DST="$2"

rosbags-convert \
  --src "$SRC" \
  --dst "$DST" \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_jazzy \
  --dst-storage mcap

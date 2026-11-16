#!/usr/bin/env bash
# Render a smooth RViz video from a rosbag, fully headless.
#
# RViz runs on a virtual X display (Xvfb), the bag is replayed slower than real
# time so every frame gets rendered, ffmpeg grabs the virtual display, and the
# result is re-timed back to real (sim) time. Your desktop is not touched, and
# it works under Wayland.
#
# Usage:
#   ./render_rviz_video.sh <bag_dir> <config.rviz> [out.mp4] [rate] [WxH] [fps]
#   ./render_rviz_video.sh run1 px4_sim_5ugv_5uav_warehouse_light/ugv_uav_pairs.rviz run1.mp4 0.25 1920x1080 30
#
# Needs: xvfb (sudo apt install xvfb), ffmpeg, xdotool, sourced ROS 2 workspace.
set -euo pipefail

BAG=${1:?bag dir}
CFG=${2:?rviz config}
OUT=${3:-rviz_$(date +%Y%m%d_%H%M%S).mp4}
RATE=${4:-0.25}      # bag playback rate; lower = smoother but slower render
SIZE=${5:-1920x1080}
FPS=${6:-30}         # output video fps
DISP=:${RVIZ_REC_DISPLAY:-99}
W=${SIZE%x*}; H=${SIZE#*x}

[ -e "$BAG/metadata.yaml" ] || { echo "No rosbag at '$BAG' (expected $BAG/metadata.yaml). Record one first: ros2 bag record -o $BAG ..."; exit 1; }
[ -f "$CFG" ] || { echo "RViz config '$CFG' not found"; exit 1; }
command -v Xvfb >/dev/null || { echo "Xvfb missing: sudo apt install xvfb"; exit 1; }
command -v ros2 >/dev/null || { echo "ros2 not found: source your workspace first"; exit 1; }
RAW=$(mktemp --suffix=.mkv)
# Capture only as many frames as the final video needs (FPS * RATE), min 2.
CAP_FPS=$(python3 -c "print(max(2.0, $FPS*$RATE))")

pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; rm -f "$RAW"; }
trap cleanup EXIT

Xvfb "$DISP" -screen 0 "${W}x${H}x24" -nolisten tcp &
pids+=($!)
sleep 1

DISPLAY=$DISP rviz2 -d "$CFG" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
pids+=($!)

# No window manager in Xvfb: force the RViz window to fill the screen.
WID=$(DISPLAY=$DISP xdotool search --sync --onlyvisible --class rviz2 | head -1)
DISPLAY=$DISP xdotool windowmove "$WID" 0 0 windowsize "$WID" "$W" "$H"
sleep 4   # let RViz load displays/meshes

ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate "$CAP_FPS" -video_size "$SIZE" \
       -i "$DISP" -c:v libx264 -preset ultrafast -crf 16 "$RAW" &
FF=$!
pids+=($FF)

echo "Replaying $BAG at ${RATE}x (capturing ${CAP_FPS} fps)..."
ros2 bag play "$BAG" --clock -r "$RATE"
sleep 1

kill -INT "$FF"; wait "$FF" || true

echo "Re-timing to real speed -> $OUT"
ffmpeg -loglevel error -y -i "$RAW" -vf "setpts=${RATE}*PTS,fps=${FPS}" \
       -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p -an "$OUT"
echo "Done: $OUT"

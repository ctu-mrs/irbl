#!/bin/bash

URL="https://nasmrs.fel.cvut.cz:3923/PERSONAL_DIRECTORIES/manuel/BAGS4GITHUB/circle.bag"
OUT_FILE="rosbag_input.bag"
MCAP_FILE="rosbag.mcap"
HOST="nasmrs.fel.cvut.cz"

# -------------------------
# Connectivity check
# -------------------------
echo "[INFO] Checking internet connectivity..."

if ! ping -c 1 -W 2 "$HOST" >/dev/null 2>&1; then
  echo "[ERROR] No internet connection or host unreachable: $HOST"
  exit 1
fi

echo "[INFO] Internet OK"

# -------------------------
# Download
# -------------------------
if [ -s "$OUT_FILE" ]; then
  echo "[INFO] Bag already exists, skipping download: $OUT_FILE"
else
  echo "[INFO] Downloading bag..."

  if ! wget -c -O "$OUT_FILE" "$URL"; then
    echo "[ERROR] Download failed"
    exit 1
  fi

  if [ ! -s "$OUT_FILE" ]; then
    echo "[ERROR] File is empty or corrupted!"
    exit 1
  fi

  echo "[INFO] Download complete"
fi

# -------------------------
# Ensure rosbags-convert exists
# -------------------------
if ! command -v rosbags-convert >/dev/null 2>&1; then
  echo "[WARN] rosbags-convert not found, installing via pipx..."

  if ! command -v pipx >/dev/null 2>&1; then
    echo "[INFO] Installing pipx..."
    sudo apt update
    sudo apt install -y pipx
    pipx ensurepath
  fi

  export PATH="$HOME/.local/bin:$PATH"

  if ! pipx list | grep -q rosbags; then
    if ! pipx install rosbags; then
      echo "[ERROR] Failed to install rosbags"
      exit 1
    fi
  fi

  export PATH="$HOME/.local/bin:$PATH"

  if ! command -v rosbags-convert >/dev/null 2>&1; then
    echo "[ERROR] rosbags-convert still not available"
    exit 1
  fi

  echo "[INFO] rosbags-convert ready"
else
  echo "[INFO] rosbags-convert already available"
fi

# -------------------------
# Detect format
# -------------------------
echo "[INFO] Detecting bag format..."

if ros2 bag info "$OUT_FILE" >/dev/null 2>&1; then
  echo "[INFO] Detected ROS2 bag"

  STORAGE=$(ros2 bag info "$OUT_FILE" | grep "Storage id" | awk '{print $3}')

  if [ "$STORAGE" = "mcap" ]; then
    echo "[INFO] Already MCAP format"
    PLAY_FILE="$OUT_FILE"
  else
    echo "[INFO] Converting ROS2 bag to MCAP..."

    rosbags-convert \
      --src "$OUT_FILE" \
      --dst "$MCAP_FILE" \
      --dst-storage mcap

    PLAY_FILE="$MCAP_FILE"
  fi

else
  echo "[INFO] Assuming ROS1 bag → converting to ROS2 MCAP..."

  rosbags-convert \
    --src "$OUT_FILE" \
    --dst "$MCAP_FILE" \
    --src-typestore ros1_noetic \
    --dst-typestore ros2_jazzy \
    --dst-storage mcap

  PLAY_FILE="$MCAP_FILE"
fi

# -------------------------
# Play
# -------------------------
echo "[INFO] Playing bag: $PLAY_FILE"
ros2 bag play "$PLAY_FILE" -r 0.1 -l

#!/bin/bash

URL="https://nasmrs.fel.cvut.cz:3923/PERSONAL_DIRECTORIES/manuel/BAGS4GITHUB/rosbag2_2026_04_29-11_59_36_0.mcap"
OUT_FILE="rosbag.mcap"
HOST="nasmrs.fel.cvut.cz"

echo "[INFO] Checking internet connectivity..."

# Try to reach the host
if ! ping -c 1 -W 2 "$HOST" >/dev/null 2>&1; then
  echo "[ERROR] No internet connection or host unreachable: $HOST"
  exit 1
fi

echo "[INFO] Internet OK"

# Check if file already exists and is non-empty
if [ -s "$OUT_FILE" ]; then
  echo "[INFO] Bag already exists, skipping download: $OUT_FILE"
else
  echo "[INFO] Downloading bag..."

  if ! wget -c -O "$OUT_FILE" "$URL"; then
    echo "[ERROR] Download failed (network issue?)"
    exit 1
  fi

  # Validate file
  if [ ! -s "$OUT_FILE" ]; then
    echo "[ERROR] File is empty or corrupted!"
    exit 1
  fi

  echo "[INFO] Bag ready: $OUT_FILE"
fi

echo "[INFO] Playing bag..."
ros2 bag play "$OUT_FILE" -r 0.1 -l

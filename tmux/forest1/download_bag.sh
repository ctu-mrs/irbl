#!/bin/bash

set -euo pipefail

URL="https://nasmrs.fel.cvut.cz:3923/PERSONAL_DIRECTORIES/manuel/rosbag2_2026_04_29-11_59_36_0.mcap"
OUT_FILE="rosbag.mcap"

if [ -f "$OUT_FILE" ]; then
  echo "[INFO] Bag already exists: $OUT_FILE"
else
  echo "[INFO] Downloading bag..."
  wget -O "$OUT_FILE" "$URL"
  echo "[INFO] Download complete."
fi

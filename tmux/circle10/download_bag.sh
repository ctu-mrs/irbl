#!/bin/bash

set -euo pipefail

URL="https://nasmrs.fel.cvut.cz:3923/PERSONAL_DIRECTORIES/manuel/BAGS4GITHUB/circle.bag"
OUT_FILE="circle.bag"

if [ -f "$OUT_FILE" ]; then
  echo "[INFO] Bag already exists: $OUT_FILE"
else
  echo "[INFO] Downloading bag..."
  wget -O "$OUT_FILE" "$URL"
  echo "[INFO] Download complete."
fi

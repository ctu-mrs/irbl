#!/bin/bash

set -euo pipefail

URL="https://nasmrs.fel.cvut.cz:3923/PERSONAL_DIRECTORIES/manuel/BAGS4GITHUB/forest.bag"
OUT_FILE="forest.bag"

if [ -f "$OUT_FILE" ]; then
  echo "[INFO] Bag already exists: $OUT_FILE"
else
  echo "[INFO] Downloading bag..."
  wget -O "$OUT_FILE" "$URL"
  echo "[INFO] Download complete."
fi

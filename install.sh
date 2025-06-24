#!/usr/bin/env bash
set -e

BOARD_PORT=${BOARD_PORT:-/dev/ttyUSB0}

# Check for git
if ! command -v git >/dev/null 2>&1; then
  echo "git not found. Please install git." >&2
  exit 1
fi

# Get ESP-IDF if not present
if [ ! -d "$HOME/esp/esp-idf" ]; then
  mkdir -p "$HOME/esp"
  git clone --recursive https://github.com/espressif/esp-idf.git "$HOME/esp/esp-idf"
fi

# Set up ESP-IDF environment
source "$HOME/esp/esp-idf/export.sh"

idf.py build
idf.py -p "$BOARD_PORT" flash

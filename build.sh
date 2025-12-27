#!/bin/bash
set -euo pipefail

mkdir -p third_party

if [ ! -f third_party/httplib.h ]; then
  echo "[build] fetching cpp-httplib..."
  curl -L --fail -o third_party/httplib.h \
    https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.15.3/httplib.h
fi

echo "[build] compiling..."
g++ -O3 -Wall -Wextra -pthread -std=c++17 \
  -o tc358743_drm_present_webui \
  tc358743_drm_present_webui.cpp \
  -I/usr/include/libdrm -I./third_party \
  -ldrm -lv4l2

echo "[build] built: ./tc358743_drm_present_webui"

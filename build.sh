#!/bin/bash
set -euo pipefail

mkdir -p third_party
if [ ! -f third_party/httplib.h ]; then
  echo "[build] fetching cpp-httplib..."
  curl -L --fail -o third_party/httplib.h \
    https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.15.3/httplib.h
fi
if [ ! -f third_party/json.hpp ]; then
  echo "[build] fetching nlohmann/json..."
  curl -L --fail -o third_party/json.hpp \
    https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp
fi

echo "[build] compiling..."
CXXFLAGS=(
  -O3
  -march=native
  -flto
  -fomit-frame-pointer
  -DNDEBUG
  -Wall -Wextra
  -pthread
  -std=c++17
  -I/usr/include/libdrm
  -I./third_party
)

LDFLAGS=(
  -flto
  -ldrm
  -lv4l2
  -pthread
)

g++ "${CXXFLAGS[@]}" \
  -o tc358743_drm_present_main \
  tc358743_drm_present_main.cpp \
  tc358743_webui.cpp \
  overlay_backend.cpp \
  v4l2_caps.cpp \
  "${LDFLAGS[@]}"

echo "[build] built: ./tc358743_drm_present_main"

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

# Keep exceptions ON because cpp-httplib uses try/catch internally.
# Still keep the speed flags that matter for your compositor.
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
)

g++ "${CXXFLAGS[@]}" \
  -o tc358743_drm_present_webui \
  tc358743_drm_present_webui.cpp \
  tc358743_webui.cpp \
  "${LDFLAGS[@]}"

echo "[build] built: ./tc358743_drm_present_webui"

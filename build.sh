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

# DepthAI is expected to be installed system-wide such that:
#   - headers are reachable (e.g. /usr/include/depthai)
#   - library is reachable (e.g. /usr/lib/aarch64-linux-gnu/libdepthai.so)
#
# If your include path differs, export:
#   DEPTHAI_INC=/path/to/depthai/include
# If your lib path differs, export:
#   DEPTHAI_LIB=/path/to/depthai/lib

DEPTHAI_INC=/media/FALCON/Luxonis/depthai-core/build/install/include
DEPTHAI_LIB=/media/FALCON/Luxonis/depthai-core/build/install/lib
DEPTHAI_DEPS_INC=/media/FALCON/Luxonis/depthai-core/build/install/lib/cmake/depthai/dependencies/include
DEPTHAI_THIRD_PARTY_INC=/media/FALCON/Luxonis/depthai-core/build/install/include/depthai-shared/3rdparty

CXXFLAGS=(
  -O3
  -march=native
  -flto=auto
  -fomit-frame-pointer
  -DNDEBUG
  -Wall -Wextra
  -pthread
  -std=c++17
  -I/usr/include/libdrm
  -I./third_party
  -I${DEPTHAI_INC}
  -I${DEPTHAI_DEPS_INC}
  -I${DEPTHAI_THIRD_PARTY_INC}
  $(pkg-config --cflags opencv4)
  $(pkg-config --cflags egl glesv2)
)

LDFLAGS=(
  -flto=auto
  -ldrm
  -lv4l2
  -pthread
  -L${DEPTHAI_LIB}
  -ldepthai-core
  $(pkg-config --libs opencv4)
  $(pkg-config --libs egl glesv2)
)

g++ "${CXXFLAGS[@]}" \
  -o tc358743_drm_present_main \
  tc358743_drm_present_main.cpp \
  tc358743_webui.cpp \
  overlay_backend.cpp \
  v4l2_caps.cpp \
  oak_accel.cpp \
  gpu_clahe.cpp \
  "${LDFLAGS[@]}"

echo "[build] built: ./tc358743_drm_present_main"

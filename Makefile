# Makefile for tc358743_drm_overlay
# Usage:
#   make	    # fetch deps + build
#   make clean
#   make distclean  # also removes downloaded third_party headers
#
# Override paths if needed, e.g.:
#   make DEPTHAI_INC=/path/include DEPTHAI_LIB=/path/lib

SHELL := /bin/bash

TARGET := tc358743_drm_present_main

SRCS := \
    tc358743_drm_present_main.cpp \
    tc358743_webui_stream_endpoints.cpp \
    tc358743_webui_globals.cpp \
    tc358743_webui.cpp \
    overlay_backend.cpp \
    v4l2_caps.cpp \
    oak_accel.cpp \
    gpu_clahe.cpp \
    jpeg_encode_fast.cpp

OBJS := $(SRCS:.cpp=.o)

THIRD_PARTY_DIR := third_party
HTTPLIB_H := $(THIRD_PARTY_DIR)/httplib.h
JSON_HPP := $(THIRD_PARTY_DIR)/json.hpp

HTTPLIB_URL := https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.15.3/httplib.h
JSON_URL := https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp

# ---- libjpeg-turbo (TurboJPEG 3.x) vendored build ----
JPEGTURBO_VER := 3.0.3
JPEGTURBO_DIR := $(THIRD_PARTY_DIR)/libjpeg-turbo
JPEGTURBO_BUILD := $(JPEGTURBO_DIR)/build
JPEGTURBO_PREFIX := $(THIRD_PARTY_DIR)/jpeg-turbo
JPEGTURBO_INC := $(JPEGTURBO_PREFIX)/include
JPEGTURBO_LIB := $(JPEGTURBO_PREFIX)/lib
JPEGTURBO_SO  := $(JPEGTURBO_LIB)/libturbojpeg.so

CXX ?= g++
CC ?= gcc

SSD_DIR := .
SSD_CSRC := \
  ssd1306.c \
  linux_i2c.c \
  ssd1306_app.c

SSD_OBJS := $(SSD_CSRC:.c=.o)

# Default DepthAI paths (matching your script). Override on make command line if desired.
DEPTHAI_INC ?= /media/FALCON/Luxonis/depthai-core/build/install/include
DEPTHAI_LIB ?= /media/FALCON/Luxonis/depthai-core/build/install/lib
DEPTHAI_DEPS_INC ?= /media/FALCON/Luxonis/depthai-core/build/install/lib/cmake/depthai/dependencies/include
DEPTHAI_THIRD_PARTY_INC ?= /media/FALCON/Luxonis/depthai-core/build/install/include/depthai-shared/3rdparty
DEPTHAI_DEPS_LIB ?= $(DEPTHAI_LIB)/cmake/depthai/dependencies/lib
DEPTHAI_XLINK_A   ?= $(DEPTHAI_DEPS_LIB)/libXLink.a
DEPTHAI_SPDLOG_A  ?= $(DEPTHAI_DEPS_LIB)/libspdlog.a
DEPTHAI_FMT_A     ?= $(DEPTHAI_DEPS_LIB)/libfmt.a
DEPTHAI_CPR_A     ?= $(DEPTHAI_DEPS_LIB)/libcpr.a
DEPTHAI_CURL_A    ?= $(DEPTHAI_DEPS_LIB)/libcurl.a
DEPTHAI_ARCHIVE_A ?= $(DEPTHAI_DEPS_LIB)/libarchive_static.a
DEPTHAI_SSL_A     ?= $(DEPTHAI_DEPS_LIB)/libssl.a
DEPTHAI_CRYPTO_A  ?= $(DEPTHAI_DEPS_LIB)/libcrypto.a
DEPTHAI_Z_A       ?= $(DEPTHAI_DEPS_LIB)/libz.a
DEPTHAI_BZ2_A     ?= $(DEPTHAI_DEPS_LIB)/libbz2.a
DEPTHAI_LZMA_A    ?= $(DEPTHAI_DEPS_LIB)/liblzma.a
DEPTHAI_RES_A ?= /media/SSK/Sources/Luxonis/depthai-core/build/libdepthai-resources.a

LIBUSB_A ?= /usr/lib/aarch64-linux-gnu/libusb-1.0.a

PKG_CFLAGS := $(shell pkg-config --cflags opencv4) $(shell pkg-config --cflags egl glesv2)
PKG_LIBS   := $(shell pkg-config --libs opencv4)   $(shell pkg-config --libs egl glesv2)

CXXFLAGS ?=
CXXFLAGS += \
    -O3 \
    -march=native \
    -flto=auto \
    -fomit-frame-pointer \
    -DNDEBUG \
    -Wall -Wextra \
    -pthread \
    -std=c++17 \
    -ffunction-sections \
    -fdata-sections \
    -Wno-unused-parameter \
    -Wno-type-limits \
    $(pkg-config --cflags libjpeg libturbojpeg) \
    -I/usr/include/libdrm \
    -I./$(THIRD_PARTY_DIR) \
    -I$(DEPTHAI_INC) \
    -I$(DEPTHAI_DEPS_INC) \
    -I$(DEPTHAI_THIRD_PARTY_INC) \
    $(PKG_CFLAGS)
CXXFLAGS += -I./$(JPEGTURBO_INC)
CXXFLAGS += -I./$(SSD_DIR) -DNMEA_MINI_ENABLE_TZ

CFLAGS ?=
CFLAGS += -O3 -DNDEBUG -Wall -Wextra -pthread -ffunction-sections -fdata-sections -flto=auto -I./$(SSD_DIR) -DNMEA_MINI_ENABLE_TZ

LDFLAGS ?=
LDFLAGS += \
    -flto=8 \
    -L$(DEPTHAI_LIB) \
    -L$(DEPTHAI_DEPS_LIB) \
    -Wl,--gc-sections
LDFLAGS += -L./$(JPEGTURBO_LIB) -Wl,-rpath,'$$ORIGIN/$(JPEGTURBO_LIB)'

LDLIBS ?=
LDLIBS += \
    -ldrm \
    -lv4l2 \
    -lz \
    -lbz2 \
    -llzma \
    -ludev \
    -lm \
    $(pkg-config --libs libjpeg libturbojpeg) \
    ./$(JPEGTURBO_LIB)/libturbojpeg.a \
    $(DEPTHAI_LIB)/libdepthai-core.a \
    -Wl,--whole-archive $(DEPTHAI_RES_A) -Wl,--no-whole-archive \
    $(DEPTHAI_XLINK_A) \
    $(LIBUSB_A) \
    $(DEPTHAI_ARCHIVE_A) \
    $(DEPTHAI_SPDLOG_A) \
    -lfmt \
    $(DEPTHAI_CPR_A) \
    $(DEPTHAI_CURL_A) \
    $(DEPTHAI_SSL_A) \
    $(DEPTHAI_CRYPTO_A) \
    -pthread \
    $(PKG_LIBS)
#LDLIBS += -lturbojpeg

# ----- Static options -----
# FULL static: forces static resolution for *everything*
ifeq ($(STATIC),1)
  LDFLAGS += -static
endif

# Mostly static: keep glibc/etc dynamic, but embed libstdc++/libgcc and allow
# static resolution where possible.
ifeq ($(MOSTLY_STATIC),1)
  CXXFLAGS += -fno-plt
  LDFLAGS  += -static-libstdc++ -static-libgcc
endif

.PHONY: all clean distclean deps

all: deps $(TARGET)
	@echo "[build] built: ./$(TARGET)"
	strip --strip-unneeded ./$(TARGET)

deps: $(HTTPLIB_H) $(JSON_HPP) $(JPEGTURBO_SO)

$(THIRD_PARTY_DIR):
	@mkdir -p $(THIRD_PARTY_DIR)

$(HTTPLIB_H): | $(THIRD_PARTY_DIR)
	@echo "[build] fetching cpp-httplib..."
	@curl -L --fail -o "$@" "$(HTTPLIB_URL)"

$(JSON_HPP): | $(THIRD_PARTY_DIR)
	@echo "[build] fetching nlohmann/json..."
	@curl -L --fail -o "$@" "$(JSON_URL)"

$(JPEGTURBO_DIR): | $(THIRD_PARTY_DIR)
	@echo "[build] fetching libjpeg-turbo $(JPEGTURBO_VER)..."
	@if [ ! -d "$(JPEGTURBO_DIR)/.git" ]; then \
		git clone --depth 1 --branch $(JPEGTURBO_VER) https://github.com/libjpeg-turbo/libjpeg-turbo.git "$(JPEGTURBO_DIR)"; \
	else \
		echo "[build] libjpeg-turbo already present"; \
	fi

$(JPEGTURBO_SO): $(JPEGTURBO_DIR)
	@echo "[build] building libjpeg-turbo..."
	@mkdir -p "$(JPEGTURBO_BUILD)"
	@cmake -G Ninja -S "$(JPEGTURBO_DIR)" -B "$(JPEGTURBO_BUILD)" \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_INSTALL_PREFIX="$(abspath $(JPEGTURBO_PREFIX))" \
		-DENABLE_SHARED=ON -DENABLE_STATIC=ON -DWITH_TURBOJPEG=ON \
		-DCMAKE_C_FLAGS="-pthread"
	@ninja -C "$(JPEGTURBO_BUILD)"
	@ninja -C "$(JPEGTURBO_BUILD)" install

$(TARGET): $(OBJS) $(SSD_OBJS)
	@echo "[build] linking..."
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

%.o: %.cpp deps
	@echo "[build] compiling $< ..."
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.c
	@echo "[build] compiling C $< ..."
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	@rm -f $(OBJS) $(SSD_OBJS) $(TARGET)

distclean: clean
	@rm -f $(HTTPLIB_H) $(JSON_HPP)
	@rmdir $(THIRD_PARTY_DIR) 2>/dev/null || true

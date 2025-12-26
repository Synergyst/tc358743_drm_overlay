#!/bin/bash

set -e

gcc -O3 -Wall -Wextra -pthread -o tc358743_drm_present tc358743_drm_present.c -I/usr/include/libdrm -ldrm -lv4l2
./tc358743_drm_present --bgr --edid /root/drm_overlay/720P60EDID.txt --v4l2-dev /dev/video0 --modeset=match-input --present=stretch --threads 4 --crosshair 50x50 --crosshair-color 255,255,255 --crosshair-thickness 1

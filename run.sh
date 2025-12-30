#!/bin/bash

cd /root/drm_overlay

#/root/drm_overlay/tc358743_drm_present_webui --listen 0.0.0.0 --webui-port 8080 --edid /root/drm_overlay/720P60EDID.txt --bgr
#/root/drm_overlay/tc358743_drm_present_main --listen 0.0.0.0 --webui-port 8080 --edid /root/drm_overlay/720P60EDID.txt --bgr --v4l2-dev /dev/v4l/by-path/platform-fe800000.csi-video-index0 --v4l2-src /dev/v4l/by-path/platform-fe800000.csi-video-index0,/dev/v4l/by-path/platform-fe801000.csi-video-index0,/dev/v4l/by-id/usb-IRay_M3Lite-video-index0
/root/drm_overlay/tc358743_drm_present_main --config /root/drm_overlay/overlay_config.json --listen 0.0.0.0 --webui-port 8080 --edid /root/drm_overlay/720P60EDID.txt --bgr --v4l2-dev /dev/v4l/by-path/platform-fe800000.csi-video-index0 --v4l2-src /dev/v4l/by-id/usb-IRay_M3Lite-video-index0
#/root/drm_overlay/tc358743_drm_present_main --config /root/drm_overlay/overlay_config.json --listen 0.0.0.0 --webui-port 8080 --edid /root/drm_overlay/720P60EDID.txt --bgr --v4l2-dev /dev/v4l/by-path/platform-fe801000.csi-video-index0 --v4l2-src /dev/v4l/by-id/usb-IRay_M3Lite-video-index0

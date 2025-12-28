#!/bin/bash

cd /root/drm_overlay

#/root/drm_overlay/tc358743_drm_present_webui --listen 0.0.0.0 --webui-port 8080 --edid /root/drm_overlay/720P60EDID.txt --bgr
/root/drm_overlay/tc358743_drm_present_main --listen 0.0.0.0 --webui-port 8080 --edid /root/drm_overlay/720P60EDID.txt --bgr

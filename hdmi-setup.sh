#!/bin/bash

ls /dev/video*
v4l2-ctl --list-devices
for m in {0,1} ; do
#for m in {2,3} ; do
  echo
  echo
  echo
  echo "HDMI ($m):"
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls ~/v4l2-video-capture-testing-program/1080P50EDID.txt` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls ~/v4l2-video-capture-testing-program/1080P60EDID.txt` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls /root/hdmi_to_csi_driver/1080p30edid` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls /root/hdmi_to_csi_driver/1080p30edid` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls /root/hdmi_to_csi_driver/720p60edid` --fix-edid-checksums
  v4l2-ctl -d /dev/v4l/by-path/platform-fe80$(echo -n $m)000.csi-video-index0 --set-edid=file=`ls /root/drm_overlay/720P60EDID.txt` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls ~/v4l2-video-capture-testing-program/customgoodhdmiedid_binary.txt` --fix-edid-checksums
  sleep 2
  v4l2-ctl -d /dev/v4l/by-path/platform-fe80$(echo -n $m)000.csi-video-index0 --query-dv-timings
  sleep 1
  v4l2-ctl -d /dev/v4l/by-path/platform-fe80$(echo -n $m)000.csi-video-index0 --set-dv-bt-timings query
  sleep 1
  v4l2-ctl -d /dev/v4l/by-path/platform-fe80$(echo -n $m)000.csi-video-index0 -V
  sleep 1
  v4l2-ctl -d /dev/v4l/by-path/platform-fe80$(echo -n $m)000.csi-video-index0 -v pixelformat=RGB3
  #v4l2-ctl -d /dev/video$m -v pixelformat=UYVY
  sleep 1
  echo -n "/dev/v4l/by-path/platform-fe80$(echo -n $m)000.csi-video-index0:"
  v4l2-ctl -d /dev/v4l/by-path/platform-fe80$(echo -n $m)000.csi-video-index0 --log-status
done

#!/bin/bash

ls /dev/video*
v4l2-ctl --list-devices
#for m in {0,1} ; do
for m in {2,3} ; do
  echo
  echo
  echo
  echo "HDMI ($m):"
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls ~/v4l2-video-capture-testing-program/1080P50EDID.txt` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls ~/v4l2-video-capture-testing-program/1080P60EDID.txt` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls /root/hdmi_to_csi_driver/1080p30edid` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls /root/hdmi_to_csi_driver/1080p30edid` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls /root/hdmi_to_csi_driver/720p60edid` --fix-edid-checksums
  v4l2-ctl -d /dev/video$m --set-edid=file=`ls /root/drm_overlay/720P60EDID.txt` --fix-edid-checksums
  #v4l2-ctl -d /dev/video$m --set-edid=file=`ls ~/v4l2-video-capture-testing-program/customgoodhdmiedid_binary.txt` --fix-edid-checksums
  sleep 2
  v4l2-ctl -d /dev/video$m --query-dv-timings
  sleep 1
  v4l2-ctl -d /dev/video$m --set-dv-bt-timings query
  sleep 1
  v4l2-ctl -d /dev/video$m -V
  sleep 1
  v4l2-ctl -d /dev/video$m -v pixelformat=RGB3
  #v4l2-ctl -d /dev/video$m -v pixelformat=UYVY
  sleep 1
  echo -n "/dev/video$m:"
  v4l2-ctl -d /dev/video$m --log-status
done

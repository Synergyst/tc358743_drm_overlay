#include "v4l2_caps.h"
#include <libv4l2.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

static int xioctl_local(int fd, int req, void *arg) {
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

v4l2_device_caps v4l2_query_caps(const std::string &dev) {
  v4l2_device_caps caps{};
  caps.dev = dev;

  int fd = v4l2_open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    caps.ok = false;
    caps.err = std::string("open failed: ") + strerror(errno);
    return caps;
  }

  struct v4l2_fmtdesc f;
  memset(&f, 0, sizeof(f));
  f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  for (f.index = 0; ; f.index++) {
    if (xioctl_local(fd, VIDIOC_ENUM_FMT, &f) < 0) break;

    v4l2_format_caps fc;
    fc.pixfmt = f.pixelformat;
    fc.desc = (const char*)f.description;

    // Enumerate discrete frame sizes (common for UVC).
    struct v4l2_frmsizeenum fs;
    memset(&fs, 0, sizeof(fs));
    fs.pixel_format = f.pixelformat;

    for (fs.index = 0; ; fs.index++) {
      if (xioctl_local(fd, VIDIOC_ENUM_FRAMESIZES, &fs) < 0) break;
      if (fs.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        fc.sizes.emplace_back(fs.discrete.width, fs.discrete.height);
      } else if (fs.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        // Keep minimal support: include min and max as "choices".
        fc.sizes.emplace_back(fs.stepwise.min_width, fs.stepwise.min_height);
        fc.sizes.emplace_back(fs.stepwise.max_width, fs.stepwise.max_height);
        break;
      } else if (fs.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
        // Not useful as-is without UI sliders; just expose nothing.
        break;
      }
    }

    caps.formats.push_back(std::move(fc));
  }

  v4l2_close(fd);
  caps.ok = true;
  return caps;
}

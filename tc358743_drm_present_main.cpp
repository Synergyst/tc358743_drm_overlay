#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <arm_neon.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <libv4l2.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <libdrm/drm.h>
#include <libdrm/drm_mode.h>
#include <libdrm/drm_fourcc.h>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include "third_party/json.hpp"
#include "tc358743_webui.h"
#include "overlay_backend.h"
#include "v4l2_caps.h"
#include "v4l2_convert.h"

using nlohmann::json;

// -------------------- Helpers --------------------
static void die(const char *msg) { perror(msg); exit(1); }
static int xioctl(int fd, int req, void *arg) {
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}
static int set_fd_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return -1;
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return -1;
  return 0;
}
static std::string slurp_file(const std::string &path) {
  std::ifstream f(path);
  if (!f.is_open()) return {};
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
static bool write_file_atomic(const std::string &path, const std::string &data) {
  std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f.is_open()) return false;
    f << data;
    f.flush();
    if (!f.good()) return false;
  }
  if (rename(tmp.c_str(), path.c_str()) != 0) return false;
  return true;
}
static uint64_t monotonic_ms() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

// ---------------- raw terminal (q to quit) ----------------
static struct termios orig_termios;
static void disableRawMode(void) { tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios); }
static void enableRawModeNonBlockingStdin(void) {
  tcgetattr(STDIN_FILENO, &orig_termios);
  atexit(disableRawMode);
  struct termios raw = orig_termios;
  raw.c_lflag &= ~(ECHO | ICANON | ISIG);
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  if (set_fd_nonblocking(STDIN_FILENO) != 0) {
    fprintf(stderr, "[warn] could not set stdin nonblocking: %s\n", strerror(errno));
  }
}

// ---------------- V4L2 capture ----------------
struct cap_buf { void *start; size_t length; };
static bool read_file_binary(const char *path, uint8_t **out_buf, size_t *out_len) {
  *out_buf = NULL; *out_len = 0;
  FILE *f = fopen(path, "rb");
  if (!f) return false;
  if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return false; }
  long len = ftell(f);
  if (len <= 0) { fclose(f); return false; }
  if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return false; }
  uint8_t *buf = (uint8_t*)malloc((size_t)len);
  if (!buf) { fclose(f); return false; }
  size_t rd = fread(buf, 1, (size_t)len, f);
  fclose(f);
  if (rd != (size_t)len) { free(buf); return false; }
  *out_buf = buf; *out_len = (size_t)len;
  return true;
}
static void fix_edid_checksums_inplace(uint8_t *edid, size_t len) {
  if (len % 128 != 0) return;
  size_t blocks = len / 128;
  for (size_t b=0;b<blocks;b++) {
    uint8_t sum = 0;
    uint8_t *blk = edid + b*128;
    for (int i=0;i<127;i++) sum = (uint8_t)(sum + blk[i]);
    blk[127] = (uint8_t)(0x100 - sum);
  }
}
static int v4l2_set_edid_if_requested(int fd, const char *edid_path) {
  if (!edid_path || !edid_path[0]) return 0;
  uint8_t *edid=NULL; size_t len=0;
  if (!read_file_binary(edid_path, &edid, &len)) return -1;
  if (len < 128 || (len % 128) != 0) { free(edid); return -1; }
  fix_edid_checksums_inplace(edid, len);
  struct v4l2_edid vedid;
  memset(&vedid, 0, sizeof(vedid));
  vedid.start_block = 0;
  vedid.blocks = (uint32_t)(len/128);
  vedid.edid = edid;
  int rc = xioctl(fd, VIDIOC_S_EDID, &vedid);
  free(edid);
  if (rc < 0) fprintf(stderr, "[v4l2] VIDIOC_S_EDID failed: %s\n", strerror(errno));
  return rc;
}
static int tc358743_query_dv_timings(int fd, uint32_t *out_w, uint32_t *out_h, uint64_t *out_pixelclock) {
  struct v4l2_dv_timings t;
  memset(&t, 0, sizeof(t));
  if (xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &t) < 0) return -1;
  if (out_w) *out_w = t.bt.width;
  if (out_h) *out_h = t.bt.height;
  if (out_pixelclock) *out_pixelclock = t.bt.pixelclock;
  return 0;
}
static int tc358743_query_and_set_dv_timings(int fd, uint32_t *out_w, uint32_t *out_h, uint64_t *out_pixelclock, bool log) {
  struct v4l2_dv_timings t;
  memset(&t, 0, sizeof(t));
  if (xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &t) < 0) {
    fprintf(stderr, "[v4l2] VIDIOC_QUERY_DV_TIMINGS failed: %s\n", strerror(errno));
    return -1;
  }
  if (xioctl(fd, VIDIOC_S_DV_TIMINGS, &t) < 0) {
    fprintf(stderr, "[v4l2] VIDIOC_S_DV_TIMINGS failed: %s\n", strerror(errno));
    return -1;
  }
  if (out_w) *out_w = t.bt.width;
  if (out_h) *out_h = t.bt.height;
  if (out_pixelclock) *out_pixelclock = t.bt.pixelclock;
  if (log) fprintf(stderr, "[v4l2] DV timings: %ux%u pixelclock=%llu\n", t.bt.width, t.bt.height, (unsigned long long)t.bt.pixelclock);
  return 0;
}
static int v4l2_get_current_fmt(int fd, uint32_t *out_w, uint32_t *out_h, uint32_t *out_stride, uint32_t *out_pixfmt) {
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) return -1;
  if (out_w) *out_w = fmt.fmt.pix.width;
  if (out_h) *out_h = fmt.fmt.pix.height;
  if (out_stride) *out_stride = fmt.fmt.pix.bytesperline;
  if (out_pixfmt) *out_pixfmt = fmt.fmt.pix.pixelformat;
  return 0;
}
static int v4l2_set_fmt_capture(int fd, uint32_t pixfmt, uint32_t w, uint32_t h, uint32_t *out_w, uint32_t *out_h, uint32_t *out_stride) {
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) return -1;
  fmt.fmt.pix.pixelformat = pixfmt;
  if (w) fmt.fmt.pix.width = w;
  if (h) fmt.fmt.pix.height = h;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) return -1;
  if (out_w) *out_w = fmt.fmt.pix.width;
  if (out_h) *out_h = fmt.fmt.pix.height;
  if (out_stride) *out_stride = fmt.fmt.pix.bytesperline;
  return 0;
}
static int tc358743_set_pixfmt_rgb24(int fd, uint32_t *out_w, uint32_t *out_h, uint32_t *out_stride) {
  return v4l2_set_fmt_capture(fd, V4L2_PIX_FMT_RGB24, 0, 0, out_w, out_h, out_stride);
}
static int v4l2_start_mmap_capture(int fd, struct cap_buf **out_bufs, uint32_t *out_nbufs) {
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 3;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    fprintf(stderr, "[v4l2] VIDIOC_REQBUFS failed: %s\n", strerror(errno));
    return -1;
  }
  if (req.count < 2) return -1;
  struct cap_buf *bufs = (struct cap_buf*)calloc(req.count, sizeof(*bufs));
  if (!bufs) return -1;
  for (uint32_t i=0;i<req.count;i++) {
    struct v4l2_buffer b;
    memset(&b, 0, sizeof(b));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) die("VIDIOC_QUERYBUF");
    bufs[i].length = b.length;
    bufs[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
    if (bufs[i].start == MAP_FAILED) die("mmap(v4l2)");
    if (xioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF");
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) die("VIDIOC_STREAMON");
  *out_bufs = bufs;
  *out_nbufs = req.count;
  return 0;
}
static void v4l2_stop_and_unmap(int fd, struct cap_buf *cbufs, uint32_t cbuf_count) {
  if (fd >= 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    (void)xioctl(fd, VIDIOC_STREAMOFF, &type);
  }
  for (uint32_t i=0;i<cbuf_count;i++) {
    if (cbufs && cbufs[i].start && cbufs[i].length) munmap(cbufs[i].start, cbufs[i].length);
  }
  free(cbufs);
}
static int v4l2_wait_for_frame(int fd, int timeout_ms) {
  struct pollfd pfd;
  memset(&pfd, 0, sizeof(pfd));
  pfd.fd = fd;
  pfd.events = POLLIN;
  int r = poll(&pfd, 1, timeout_ms);
  if (r < 0) {
    if (errno == EINTR) return 1;
    return -1;
  }
  if (r == 0) return 0;
  return 1;
}
static int v4l2_dequeue_frame(int fd, struct v4l2_buffer *out_b) {
  struct v4l2_buffer b;
  memset(&b, 0, sizeof(b));
  b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  b.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) {
    if (errno == EAGAIN) return 1;
    return -1;
  }
  *out_b = b;
  return 0;
}
static int v4l2_queue_frame(int fd, struct v4l2_buffer *b) {
  if (xioctl(fd, VIDIOC_QBUF, b) < 0) return -1;
  return 0;
}
static int v4l2_dequeue_latest_frame(int fd, struct v4l2_buffer *out_b) {
  struct v4l2_buffer newest;
  bool have=false;
  for (;;) {
    struct v4l2_buffer b;
    int rc = v4l2_dequeue_frame(fd, &b);
    if (rc == 0) {
      if (have) (void)v4l2_queue_frame(fd, &newest);
      newest = b;
      have = true;
      continue;
    }
    if (rc == 1) break;
    return -1;
  }
  if (!have) return 1;
  *out_b = newest;
  return 0;
}

// ---- Multi-source probing helpers (keep old behavior) ----
struct source_probe {
  bool present = false;
  bool active = false;
  uint32_t dv_w = 0, dv_h = 0;
  uint64_t pixelclock = 0;
  uint32_t fmt_w = 0, fmt_h = 0;
  uint32_t stride = 0;
  uint32_t pixfmt = 0;
  std::string err;
};
static source_probe probe_v4l2_source(const std::string &dev) {
  source_probe p{};
  int fd = v4l2_open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    p.present = false;
    p.active = false;
    p.err = std::string("open failed: ") + strerror(errno);
    return p;
  }
  p.present = true;
  uint32_t tw=0, th=0; uint64_t pc=0;
  if (tc358743_query_dv_timings(fd, &tw, &th, &pc) == 0 && tw > 0 && th > 0) {
    p.active = true;
    p.dv_w = tw;
    p.dv_h = th;
    p.pixelclock = pc;
  } else {
    // For aux devices DV timings may not exist; still consider them "present".
    p.active = true;
  }
  uint32_t fw=0, fh=0, st=0, pf=0;
  if (v4l2_get_current_fmt(fd, &fw, &fh, &st, &pf) == 0) {
    p.fmt_w = fw;
    p.fmt_h = fh;
    p.stride = st;
    p.pixfmt = pf;
  }
  v4l2_close(fd);
  return p;
}

// ---------------- DRM scanout helpers ----------------
// (Kept inline here; unchanged from your code)
struct dumb_fb {
  uint32_t fb_id;
  uint32_t handle;
  uint32_t pitch;
  uint64_t size;
  void *map;
  uint32_t width, height;
  uint32_t format;
};
static int dumb_fb_create(int drm_fd, uint32_t w, uint32_t h, uint32_t format, struct dumb_fb *out) {
  memset(out, 0, sizeof(*out));
  out->width = w; out->height = h; out->format = format;
  struct drm_mode_create_dumb creq;
  memset(&creq, 0, sizeof(creq));
  creq.width = w; creq.height = h; creq.bpp = 32;
  if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) return -1;
  out->handle = creq.handle;
  out->pitch = creq.pitch;
  out->size = creq.size;
  uint32_t handles[4] = { out->handle, 0, 0, 0 };
  uint32_t pitches[4] = { out->pitch, 0, 0, 0 };
  uint32_t offsets[4] = { 0, 0, 0, 0 };
  int ret = drmModeAddFB2(drm_fd, w, h, format, handles, pitches, offsets, &out->fb_id, 0);
  if (ret) {
    ret = drmModeAddFB(drm_fd, w, h, 24, 32, out->pitch, out->handle, &out->fb_id);
    if (ret) return -1;
  }
  struct drm_mode_map_dumb mreq;
  memset(&mreq, 0, sizeof(mreq));
  mreq.handle = out->handle;
  if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) return -1;
  out->map = mmap(0, out->size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, mreq.offset);
  if (out->map == MAP_FAILED) { out->map=NULL; return -1; }
  memset(out->map, 0, out->size);
  return 0;
}
static void dumb_fb_destroy(int drm_fd, struct dumb_fb *fb) {
  if (!fb) return;
  if (fb->map) munmap(fb->map, fb->size);
  if (fb->fb_id) drmModeRmFB(drm_fd, fb->fb_id);
  if (fb->handle) {
    struct drm_mode_destroy_dumb dreq;
    memset(&dreq, 0, sizeof(dreq));
    dreq.handle = fb->handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
  }
  memset(fb, 0, sizeof(*fb));
}
static uint32_t get_prop_id(int drm_fd, uint32_t obj_id, uint32_t obj_type, const char *name) {
  drmModeObjectProperties *props = drmModeObjectGetProperties(drm_fd, obj_id, obj_type);
  if (!props) return 0;
  uint32_t prop_id = 0;
  for (uint32_t i=0;i<props->count_props;i++) {
    drmModePropertyRes *p = drmModeGetProperty(drm_fd, props->props[i]);
    if (p) {
      if (strcmp(p->name, name) == 0) { prop_id = p->prop_id; drmModeFreeProperty(p); break; }
      drmModeFreeProperty(p);
    }
  }
  drmModeFreeObjectProperties(props);
  return prop_id;
}
static int open_vc4_card(void) {
  int fd = open("/dev/dri/card1", O_RDWR | O_CLOEXEC);
  if (fd >= 0) return fd;
  return open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
}
static bool plane_supports_format(drmModePlane *p, uint32_t fmt) {
  for (uint32_t i=0;i<p->count_formats;i++) if (p->formats[i] == fmt) return true;
  return false;
}
static uint64_t get_plane_prop_value_u64(int drm_fd, uint32_t plane_id, uint32_t prop_id, bool *ok) {
  *ok=false;
  drmModeObjectProperties *props = drmModeObjectGetProperties(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE);
  if (!props) return 0;
  uint64_t val=0;
  for (uint32_t i=0;i<props->count_props;i++) {
    if (props->props[i] == prop_id) { val = props->prop_values[i]; *ok=true; break; }
  }
  drmModeFreeObjectProperties(props);
  return val;
}
static uint32_t find_primary_plane_on_crtc(int drm_fd, uint32_t crtc_index, uint32_t format) {
  drmModePlaneRes *pres = drmModeGetPlaneResources(drm_fd);
  if (!pres) return 0;
  for (uint32_t i=0;i<pres->count_planes;i++) {
    drmModePlane *p = drmModeGetPlane(drm_fd, pres->planes[i]);
    if (!p) continue;
    if (!(p->possible_crtcs & (1u<<crtc_index))) { drmModeFreePlane(p); continue; }
    if (!plane_supports_format(p, format)) { drmModeFreePlane(p); continue; }
    uint32_t type_prop = get_prop_id(drm_fd, p->plane_id, DRM_MODE_OBJECT_PLANE, "type");
    if (type_prop) {
      bool ok=false;
      uint64_t type_val = get_plane_prop_value_u64(drm_fd, p->plane_id, type_prop, &ok);
      if (ok && type_val == 1) {
        uint32_t id = p->plane_id;
        drmModeFreePlane(p);
        drmModeFreePlaneResources(pres);
        return id;
      }
    }
    drmModeFreePlane(p);
  }
  drmModeFreePlaneResources(pres);
  return 0;
}
static int find_hdmi_a_1(int drm_fd, uint32_t *out_conn_id, drmModeConnector **out_conn) {
  drmModeRes *res = drmModeGetResources(drm_fd);
  if (!res) return -1;
  for (int i=0;i<res->count_connectors;i++) {
    drmModeConnector *c = drmModeGetConnector(drm_fd, res->connectors[i]);
    if (!c) continue;
    if (c->connector_type == DRM_MODE_CONNECTOR_HDMIA && c->connector_type_id == 1) {
      *out_conn_id = c->connector_id;
      *out_conn = c;
      drmModeFreeResources(res);
      return 0;
    }
    drmModeFreeConnector(c);
  }
  drmModeFreeResources(res);
  return -1;
}
static int get_active_crtc_for_connector(int drm_fd, drmModeConnector *conn,
                                         uint32_t *out_crtc_id, uint32_t *out_crtc_index,
                                         uint32_t *out_w, uint32_t *out_h) {
  drmModeRes *res = drmModeGetResources(drm_fd);
  if (!res) return -1;
  if (!conn->encoder_id) { drmModeFreeResources(res); return -1; }
  drmModeEncoder *enc = drmModeGetEncoder(drm_fd, conn->encoder_id);
  if (!enc) { drmModeFreeResources(res); return -1; }
  uint32_t chosen_crtc=0, chosen_index=0;
  uint32_t chosen_w=0, chosen_h=0;
  for (int i=0;i<res->count_crtcs;i++) {
    if (!(enc->possible_crtcs & (1<<i))) continue;
    uint32_t cid = res->crtcs[i];
    drmModeCrtc *crtc = drmModeGetCrtc(drm_fd, cid);
    if (!crtc) continue;
    if (crtc->mode_valid && crtc->mode.hdisplay>0 && crtc->mode.vdisplay>0) {
      chosen_crtc=cid; chosen_index=(uint32_t)i;
      chosen_w=(uint32_t)crtc->mode.hdisplay; chosen_h=(uint32_t)crtc->mode.vdisplay;
      drmModeFreeCrtc(crtc);
      break;
    }
    drmModeFreeCrtc(crtc);
    if (!chosen_crtc) { chosen_crtc=cid; chosen_index=(uint32_t)i; }
  }
  drmModeFreeEncoder(enc);
  drmModeFreeResources(res);
  if (!chosen_crtc || chosen_w==0 || chosen_h==0) return -1;
  *out_crtc_id=chosen_crtc; *out_crtc_index=chosen_index; *out_w=chosen_w; *out_h=chosen_h;
  return 0;
}
static bool connector_find_mode(drmModeConnector *conn, uint32_t w, uint32_t h, double vrefresh_hz, drmModeModeInfo *out_mode) {
  if (!conn || conn->count_modes <= 0) return false;
  for (int i=0;i<conn->count_modes;i++) {
    drmModeModeInfo m = conn->modes[i];
    if ((uint32_t)m.hdisplay != w || (uint32_t)m.vdisplay != h) continue;
    double refresh = 0.0;
    if (m.htotal>0 && m.vtotal>0) refresh = (double)m.clock*1000.0 / ((double)m.htotal*(double)m.vtotal);
    if (vrefresh_hz > 0.0) {
      if (refresh > (vrefresh_hz - 1.0) && refresh < (vrefresh_hz + 1.0)) { *out_mode = m; return true; }
    } else { *out_mode = m; return true; }
  }
  return false;
}
static int drm_modeset_match_input(int drm_fd, uint32_t conn_id, drmModeConnector *conn, uint32_t crtc_id,
                                   uint32_t want_w, uint32_t want_h) {
  drmModeModeInfo chosen;
  memset(&chosen, 0, sizeof(chosen));
  if (!connector_find_mode(conn, want_w, want_h, 60.0, &chosen) &&
      !connector_find_mode(conn, want_w, want_h, -1.0, &chosen)) {
    fprintf(stderr, "[modeset] No connector mode for %ux%u\n", want_w, want_h);
    return -1;
  }
  uint32_t mode_blob=0;
  if (drmModeCreatePropertyBlob(drm_fd, &chosen, sizeof(chosen), &mode_blob) != 0) return -1;
  uint32_t conn_crtc_id_prop = get_prop_id(drm_fd, conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
  uint32_t crtc_mode_id_prop = get_prop_id(drm_fd, crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
  uint32_t crtc_active_prop  = get_prop_id(drm_fd, crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
  drmModeAtomicReq *req = drmModeAtomicAlloc();
  if (!req) die("drmModeAtomicAlloc");
  drmModeAtomicAddProperty(req, conn_id, conn_crtc_id_prop, crtc_id);
  drmModeAtomicAddProperty(req, crtc_id, crtc_mode_id_prop, mode_blob);
  drmModeAtomicAddProperty(req, crtc_id, crtc_active_prop, 1);
  int ret = drmModeAtomicCommit(drm_fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
  drmModeAtomicFree(req);
  drmModeDestroyPropertyBlob(drm_fd, mode_blob);
  return ret;
}
static int drm_modeset_prefer(int drm_fd, uint32_t conn_id, drmModeConnector *conn, uint32_t crtc_id,
                             uint32_t prefer_w, uint32_t prefer_h, double prefer_hz) {
  drmModeModeInfo chosen;
  memset(&chosen, 0, sizeof(chosen));
  if (!connector_find_mode(conn, prefer_w, prefer_h, prefer_hz, &chosen) &&
      !connector_find_mode(conn, prefer_w, prefer_h, -1.0, &chosen)) {
    fprintf(stderr, "[modeset] preferred mode %ux%u not available\n", prefer_w, prefer_h);
    return 0;
  }
  uint32_t mode_blob=0;
  if (drmModeCreatePropertyBlob(drm_fd, &chosen, sizeof(chosen), &mode_blob) != 0) return -1;
  uint32_t conn_crtc_id_prop = get_prop_id(drm_fd, conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
  uint32_t crtc_mode_id_prop = get_prop_id(drm_fd, crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
  uint32_t crtc_active_prop  = get_prop_id(drm_fd, crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
  drmModeAtomicReq *req = drmModeAtomicAlloc();
  if (!req) die("drmModeAtomicAlloc");
  drmModeAtomicAddProperty(req, conn_id, conn_crtc_id_prop, crtc_id);
  drmModeAtomicAddProperty(req, crtc_id, crtc_mode_id_prop, mode_blob);
  drmModeAtomicAddProperty(req, crtc_id, crtc_active_prop, 1);
  int ret = drmModeAtomicCommit(drm_fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
  drmModeAtomicFree(req);
  drmModeDestroyPropertyBlob(drm_fd, mode_blob);
  return ret;
}

// ---------------- Present rectangle ----------------
struct present_rect {
  int32_t crtc_x, crtc_y;
  uint32_t crtc_w, crtc_h;
  uint32_t src_w, src_h;
};
static present_rect compute_present_rect(const char *policy, uint32_t out_w, uint32_t out_h, uint32_t in_w, uint32_t in_h) {
  present_rect r; memset(&r, 0, sizeof(r));
  r.src_w=in_w; r.src_h=in_h;
  if (strcmp(policy, "stretch")==0) { r.crtc_x=0;r.crtc_y=0;r.crtc_w=out_w;r.crtc_h=out_h; return r; }
  if (strcmp(policy, "1:1")==0) {
    r.crtc_w=in_w; r.crtc_h=in_h;
    r.crtc_x=(int32_t)((int)out_w-(int)r.crtc_w)/2;
    r.crtc_y=(int32_t)((int)out_h-(int)r.crtc_h)/2;
    return r;
  }
  double sx = in_w ? ((double)out_w/(double)in_w) : 1.0;
  double sy = in_h ? ((double)out_h/(double)in_h) : 1.0;
  double s = (sx<sy)?sx:sy;
  if (s<=0.0) s=1.0;
  uint32_t w=(uint32_t)((double)in_w*s + 0.5);
  uint32_t h=(uint32_t)((double)in_h*s + 0.5);
  if (w>out_w) w=out_w;
  if (h>out_h) h=out_h;
  r.crtc_w=w; r.crtc_h=h;
  r.crtc_x=(int32_t)((int)out_w-(int)w)/2;
  r.crtc_y=(int32_t)((int)out_h-(int)h)/2;
  return r;
}
static int drm_commit_plane_present(
  int drm_fd,
  uint32_t plane_id, uint32_t crtc_id,
  uint32_t prop_fb_id, uint32_t prop_crtc_id,
  uint32_t prop_crtc_x, uint32_t prop_crtc_y, uint32_t prop_crtc_w, uint32_t prop_crtc_h,
  uint32_t prop_src_x, uint32_t prop_src_y, uint32_t prop_src_w, uint32_t prop_src_h,
  uint32_t fb_id_to_use,
  const present_rect *pr
) {
  drmModeAtomicReq *req = drmModeAtomicAlloc();
  if (!req) die("drmModeAtomicAlloc");
  drmModeAtomicAddProperty(req, plane_id, prop_fb_id, fb_id_to_use);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_id, crtc_id);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_x, (uint32_t)pr->crtc_x);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_y, (uint32_t)pr->crtc_y);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_w, pr->crtc_w);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_h, pr->crtc_h);
  drmModeAtomicAddProperty(req, plane_id, prop_src_x, 0<<16);
  drmModeAtomicAddProperty(req, plane_id, prop_src_y, 0<<16);
  drmModeAtomicAddProperty(req, plane_id, prop_src_w, pr->src_w<<16);
  drmModeAtomicAddProperty(req, plane_id, prop_src_h, pr->src_h<<16);
  int ret = drmModeAtomicCommit(drm_fd, req, 0, NULL);
  drmModeAtomicFree(req);
  return ret;
}
static int drm_plane_flip_fb_only(int drm_fd, uint32_t plane_id, uint32_t prop_fb_id, uint32_t fb_id) {
  drmModeAtomicReq *req = drmModeAtomicAlloc();
  if (!req) return -1;
  drmModeAtomicAddProperty(req, plane_id, prop_fb_id, fb_id);
  int ret = drmModeAtomicCommit(drm_fd, req, 0, NULL);
  drmModeAtomicFree(req);
  return ret;
}

// ---------------- Compositor essentials (use XRGB sources) ----------------
static inline uint32_t invert_xrgb8888(uint32_t xrgb) {
  return (xrgb & 0xFF000000u) | ((xrgb ^ 0x00FFFFFFu) & 0x00FFFFFFu);
}
static inline uint32_t blend_over_xrgb(uint32_t under, uint32_t over, uint8_t a) {
  if (a==0) return under;
  if (a==255) return over;
  uint32_t ur=(under>>16)&0xFF, ug=(under>>8)&0xFF, ub=under&0xFF;
  uint32_t orr=(over>>16)&0xFF, og=(over>>8)&0xFF, ob=over&0xFF;
  uint32_t rr=(orr*a + ur*(255-a) + 127)/255;
  uint32_t gg=(og*a  + ug*(255-a) + 127)/255;
  uint32_t bb=(ob*a  + ub*(255-a) + 127)/255;
  return 0xFF000000u | (rr<<16) | (gg<<8) | bb;
}
struct rect_i32 { int32_t x0=0, y0=0, x1=0, y1=0; };
static inline rect_i32 rect_intersect(const rect_i32 &a, const rect_i32 &b) {
  rect_i32 r;
  r.x0 = std::max(a.x0, b.x0);
  r.y0 = std::max(a.y0, b.y0);
  r.x1 = std::min(a.x1, b.x1);
  r.y1 = std::min(a.y1, b.y1);
  if (r.x1 < r.x0) r.x1 = r.x0;
  if (r.y1 < r.y0) r.y1 = r.y0;
  return r;
}
static inline bool rect_empty(const rect_i32 &r) {
  return (r.x0 >= r.x1) || (r.y0 >= r.y1);
}
static inline int32_t map_vx_to_ox(const present_rect &pr, uint32_t vpw, int32_t vx) {
  return pr.crtc_x + (int32_t)(((int64_t)vx * (int64_t)pr.crtc_w) / (int64_t)vpw);
}
static inline int32_t map_vy_to_oy(const present_rect &pr, uint32_t vph, int32_t vy) {
  return pr.crtc_y + (int32_t)(((int64_t)vy * (int64_t)pr.crtc_h) / (int64_t)vph);
}
static inline rect_i32 layer_vp_rect_to_output_bounds(
  const present_rect &pr, uint32_t vpw, uint32_t vph,
  int32_t layer_vp_x0, int32_t layer_vp_y0, int32_t layer_vp_w,  int32_t layer_vp_h
) {
  rect_i32 r{0,0,0,0};
  if (vpw == 0 || vph == 0) return r;
  if (pr.crtc_w == 0 || pr.crtc_h == 0) return r;
  if (layer_vp_w <= 0 || layer_vp_h <= 0) return r;
  int32_t ox0 = map_vx_to_ox(pr, vpw, layer_vp_x0);
  int32_t ox1 = map_vx_to_ox(pr, vpw, layer_vp_x0 + layer_vp_w);
  int32_t oy0 = map_vy_to_oy(pr, vph, layer_vp_y0);
  int32_t oy1 = map_vy_to_oy(pr, vph, layer_vp_y0 + layer_vp_h);
  if (ox1 < ox0) std::swap(ox0, ox1);
  if (oy1 < oy0) std::swap(oy0, oy1);
  ox0 -= 1; oy0 -= 1;
  ox1 += 1; oy1 += 1;
  r.x0 = ox0; r.y0 = oy0; r.x1 = ox1; r.y1 = oy1;
  return r;
}
static inline rect_i32 clamp_bounds_to_output_and_band(const rect_i32 &r, uint32_t out_w, uint32_t out_h, uint32_t y0, uint32_t y1) {
  rect_i32 out = r;
  rect_i32 out_full{0,0,(int32_t)out_w,(int32_t)out_h};
  rect_i32 band{0,(int32_t)y0,(int32_t)out_w,(int32_t)y1};
  out = rect_intersect(out, out_full);
  out = rect_intersect(out, band);
  return out;
}
struct out_to_vp_lut {
  uint32_t out_w=0, out_h=0;
  uint32_t vpw=0, vph=0;
  present_rect pr{};
  std::vector<int32_t> ox_to_vx;
  std::vector<int32_t> oy_to_vy;
  void rebuild(uint32_t outW, uint32_t outH, uint32_t vpW, uint32_t vpH, const present_rect &p) {
    out_w=outW; out_h=outH; vpw=vpW; vph=vpH; pr=p;
    ox_to_vx.assign(out_w, -1);
    oy_to_vy.assign(out_h, -1);
    if (pr.crtc_w == 0 || pr.crtc_h == 0 || vpw == 0 || vph == 0) return;
    for (uint32_t ox=0; ox<out_w; ox++) {
      int32_t tx = (int32_t)ox - pr.crtc_x;
      if (tx < 0 || tx >= (int32_t)pr.crtc_w) continue;
      int32_t vx = (int32_t)(((int64_t)tx * (int64_t)vpw) / (int64_t)pr.crtc_w);
      if (vx < 0 || vx >= (int32_t)vpw) continue;
      ox_to_vx[ox] = vx;
    }
    for (uint32_t oy=0; oy<out_h; oy++) {
      int32_t ty = (int32_t)oy - pr.crtc_y;
      if (ty < 0 || ty >= (int32_t)pr.crtc_h) continue;
      int32_t vy = (int32_t)(((int64_t)ty * (int64_t)vph) / (int64_t)pr.crtc_h);
      if (vy < 0 || vy >= (int32_t)vph) continue;
      oy_to_vy[oy] = vy;
    }
  }
};
struct layer_map_cache {
  rect_u32 sr{};
  int32_t dst_x=0, dst_y=0;
  int32_t vp_w=0, vp_h=0;
  uint8_t alpha=255;
  layer_invert_rel inv=INV_NONE;
  bool enabled=false;
  std::vector<uint16_t> x_map;
  std::vector<uint16_t> y_map;
  void build_from_layer(const layer_cfg &L, uint32_t in_w, uint32_t in_h) {
    enabled=false;
    x_map.clear(); y_map.clear();
    if (!L.enabled || L.type != LAYER_VIDEO) return;
    rect_u32 s = L.src_rect;
    if (s.w==0 || s.h==0) return;
    if (s.x >= in_w || s.y >= in_h) return;
    if (s.x + s.w > in_w) s.w = in_w - s.x;
    if (s.y + s.h > in_h) s.h = in_h - s.y;
    if (s.w==0 || s.h==0) return;
    float sx = L.scale_x, sy = L.scale_y;
    if (!(sx > 0.0001f && sy > 0.0001f)) return;
    sr=s;
    dst_x=L.dst_pos.x; dst_y=L.dst_pos.y;
    vp_w=(int32_t)std::max(1.0f, std::floor((float)sr.w * sx + 0.5f));
    vp_h=(int32_t)std::max(1.0f, std::floor((float)sr.h * sy + 0.5f));
    alpha=(uint8_t)std::lround(std::clamp(L.opacity,0.0f,1.0f)*255.0f);
    inv=L.invert_rel;
    if (alpha==0) return;
    x_map.resize((size_t)vp_w);
    y_map.resize((size_t)vp_h);
    for (int32_t vx=0; vx<vp_w; vx++) {
      uint32_t sxi = (uint32_t)(((uint64_t)(uint32_t)vx * (uint64_t)sr.w) / (uint64_t)vp_w);
      if (sxi >= sr.w) sxi = sr.w-1;
      x_map[(size_t)vx] = (uint16_t)sxi;
    }
    for (int32_t vy=0; vy<vp_h; vy++) {
      uint32_t syi = (uint32_t)(((uint64_t)(uint32_t)vy * (uint64_t)sr.h) / (uint64_t)vp_h);
      if (syi >= sr.h) syi = sr.h-1;
      y_map[(size_t)vy] = (uint16_t)syi;
    }
    enabled=true;
  }
};
static inline void clear_xrgb_rows_fast(uint8_t *dst_xrgb, uint32_t dst_stride, uint32_t out_w, uint32_t y0, uint32_t y1) {
  for (uint32_t y=y0;y<y1;y++) {
    uint32_t *d = (uint32_t*)(dst_xrgb + (uint64_t)y*dst_stride);
    for (uint32_t x=0; x<out_w; x++) d[x] = 0xFF000000u;
  }
}

// ---------------- Minimal filters (unchanged core) ----------------
struct layer_buf {
  uint32_t w=0, h=0;
  std::vector<uint32_t> px;
  void resize(uint32_t W, uint32_t H) {
    w=W; h=H;
    px.resize((size_t)w*(size_t)h);
  }
};
static inline uint8_t luma_u8_from_xrgb(uint32_t p) {
  uint32_t r=(p>>16)&0xFF, g=(p>>8)&0xFF, b=p&0xFF;
  return (uint8_t)((r*77 + g*150 + b*29 + 128) >> 8);
}
static void filter_apply_mono(layer_buf &buf, float strength) {
  strength = std::clamp(strength, 0.0f, 1.0f);
  if (strength <= 0.0f) return;
  if (strength >= 1.0f) {
    for (size_t i=0;i<buf.px.size();i++) {
      uint32_t p = buf.px[i];
      uint8_t y = luma_u8_from_xrgb(p);
      buf.px[i] = 0xFF000000u | ((uint32_t)y<<16) | ((uint32_t)y<<8) | (uint32_t)y;
    }
    return;
  }
  const uint32_t a = (uint32_t)std::lround(strength * 255.0f);
  for (size_t i=0;i<buf.px.size();i++) {
    uint32_t p = buf.px[i];
    uint32_t r=(p>>16)&0xFF, g=(p>>8)&0xFF, b=p&0xFF;
    uint8_t y = (uint8_t)((r*77 + g*150 + b*29 + 128) >> 8);
    uint32_t rr = ((uint32_t)y*a + r*(255-a) + 127)/255;
    uint32_t gg = ((uint32_t)y*a + g*(255-a) + 127)/255;
    uint32_t bb = ((uint32_t)y*a + b*(255-a) + 127)/255;
    buf.px[i] = 0xFF000000u | (rr<<16) | (gg<<8) | bb;
  }
}
/*static void filter_apply_sobel(layer_buf &buf, filter_cfg::sobel_mode_t mode, uint8_t threshold, float alpha_f, bool invert) {
  if (buf.w < 3 || buf.h < 3) {
    if (mode == filter_cfg::SOBEL_EDGES_ONLY) std::fill(buf.px.begin(), buf.px.end(), 0x00000000u);
    else std::fill(buf.px.begin(), buf.px.end(), 0xFF000000u);
    return;
  }
  alpha_f = std::clamp(alpha_f, 0.0f, 1.0f);
  uint8_t edge_a = (uint8_t)std::lround(alpha_f * 255.0f);
  std::vector<uint8_t> lum((size_t)buf.w*(size_t)buf.h);
  for (size_t i=0;i<buf.px.size();i++) lum[i] = luma_u8_from_xrgb(buf.px[i]);
  std::vector<uint32_t> out(buf.px.size());
  auto idx = [&](int x,int y)->size_t { return (size_t)y*(size_t)buf.w + (size_t)x; };
  for (uint32_t x=0;x<buf.w;x++) { out[idx((int)x,0)] = 0; out[idx((int)x,(int)buf.h-1)] = 0; }
  for (uint32_t y=0;y<buf.h;y++) { out[idx(0,(int)y)] = 0; out[idx((int)buf.w-1,(int)y)] = 0; }
  for (uint32_t y=1; y+1<buf.h; y++) {
    for (uint32_t x=1; x+1<buf.w; x++) {
      int xm1=(int)x-1, xp1=(int)x+1;
      int ym1=(int)y-1, yp1=(int)y+1;
      int p00 = lum[idx(xm1,ym1)];
      int p01 = lum[idx((int)x,ym1)];
      int p02 = lum[idx(xp1,ym1)];
      int p10 = lum[idx(xm1,(int)y)];
      int p12 = lum[idx(xp1,(int)y)];
      int p20 = lum[idx(xm1,yp1)];
      int p21 = lum[idx((int)x,yp1)];
      int p22 = lum[idx(xp1,yp1)];
      int gx = (-p00 + p02) + (-2*p10 + 2*p12) + (-p20 + p22);
      int gy = (-p00 -2*p01 -p02) + (p20 +2*p21 + p22);
      int mag = abs(gx) + abs(gy);
      if (mag > 255) mag = 255;
      uint8_t m = (uint8_t)mag;
      if (invert) m = (uint8_t)(255 - m);
      if (mode == filter_cfg::SOBEL_EDGES_ONLY) {
        if (m < threshold || edge_a == 0) out[idx((int)x,(int)y)] = 0x00000000u;
        else out[idx((int)x,(int)y)] = ((uint32_t)edge_a<<24) | ((uint32_t)m<<16) | ((uint32_t)m<<8) | (uint32_t)m;
      } else {
        out[idx((int)x,(int)y)] = 0xFF000000u | ((uint32_t)m<<16) | ((uint32_t)m<<8) | (uint32_t)m;
      }
    }
  }
  buf.px.swap(out);
}*/
static void filter_apply_sobel(layer_buf &buf, filter_cfg::sobel_mode_t mode, uint8_t threshold, float alpha_f, bool invert) {
  if (buf.w < 3 || buf.h < 3) {
    if (mode == filter_cfg::SOBEL_EDGES_ONLY) std::fill(buf.px.begin(), buf.px.end(), 0x00000000u);
    else std::fill(buf.px.begin(), buf.px.end(), 0xFF000000u);
    return;
  }

  alpha_f = std::clamp(alpha_f, 0.0f, 1.0f);
  uint8_t edge_a = (uint8_t)std::lround(alpha_f * 255.0f);

  const uint32_t w = buf.w;
  const uint32_t h = buf.h;

  // 1) Luma
  std::vector<uint8_t> lum((size_t)w * (size_t)h);
  for (size_t i=0; i<buf.px.size(); i++) lum[i] = luma_u8_from_xrgb(buf.px[i]);

  // 2) Output
  std::vector<uint32_t> out(buf.px.size());

  auto idx = [&](uint32_t x, uint32_t y)->size_t { return (size_t)y*(size_t)w + (size_t)x; };

  // borders = 0 (matches your current behavior)
  for (uint32_t x=0; x<w; x++) { out[idx(x,0)] = 0; out[idx(x,h-1)] = 0; }
  for (uint32_t y=0; y<h; y++) { out[idx(0,y)] = 0; out[idx(w-1,y)] = 0; }

  // 3) Interior Sobel (NEON)
  for (uint32_t y=1; y+1<h; y++) {
    const uint8_t *r0 = &lum[(size_t)(y-1)*w];
    const uint8_t *r1 = &lum[(size_t)(y)*w];
    const uint8_t *r2 = &lum[(size_t)(y+1)*w];
    uint32_t *dst = &out[(size_t)y*w];

    uint32_t x = 1;

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    // Process 16 pixels at a time for x in [1 .. w-2]
    // We compute:
    // gx = (-p00 + p02) + (-2*p10 + 2*p12) + (-p20 + p22)
    // gy = (-p00 -2*p01 -p02) + (p20 +2*p21 + p22)
    // mag = abs(gx) + abs(gy), clamped to 255
    //
    // Load windows via 3 loads per row: x-1, x, x+1
    for (; x + 16 <= w-1; x += 16) {
      uint8x16_t p00 = vld1q_u8(r0 + x - 1);
      uint8x16_t p01 = vld1q_u8(r0 + x);
      uint8x16_t p02 = vld1q_u8(r0 + x + 1);

      uint8x16_t p10 = vld1q_u8(r1 + x - 1);
      uint8x16_t p12 = vld1q_u8(r1 + x + 1);

      uint8x16_t p20 = vld1q_u8(r2 + x - 1);
      uint8x16_t p21 = vld1q_u8(r2 + x);
      uint8x16_t p22 = vld1q_u8(r2 + x + 1);

      // widen to signed 16
      int16x8_t p00l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p00)));
      int16x8_t p01l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p01)));
      int16x8_t p02l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p02)));
      int16x8_t p10l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p10)));
      int16x8_t p12l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p12)));
      int16x8_t p20l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p20)));
      int16x8_t p21l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p21)));
      int16x8_t p22l = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(p22)));

      int16x8_t p00h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p00)));
      int16x8_t p01h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p01)));
      int16x8_t p02h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p02)));
      int16x8_t p10h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p10)));
      int16x8_t p12h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p12)));
      int16x8_t p20h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p20)));
      int16x8_t p21h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p21)));
      int16x8_t p22h = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(p22)));

      auto sobel8 = [&](int16x8_t p00, int16x8_t p01, int16x8_t p02,
                        int16x8_t p10,              int16x8_t p12,
                        int16x8_t p20, int16x8_t p21, int16x8_t p22) -> uint8x8_t {
        // gx = (p02 - p00) + 2*(p12 - p10) + (p22 - p20)
        int16x8_t gx = vsubq_s16(p02, p00);
        gx = vaddq_s16(gx, vshlq_n_s16(vsubq_s16(p12, p10), 1));
        gx = vaddq_s16(gx, vsubq_s16(p22, p20));

        // gy = (p20 - p00) + 2*(p21 - p01) + (p22 - p02)
        int16x8_t gy = vsubq_s16(p20, p00);
        gy = vaddq_s16(gy, vshlq_n_s16(vsubq_s16(p21, p01), 1));
        gy = vaddq_s16(gy, vsubq_s16(p22, p02));

        // abs(gx), abs(gy)
        int16x8_t ax = vabsq_s16(gx);
        int16x8_t ay = vabsq_s16(gy);

        // mag = ax + ay (still fits in 16-bit: max 2040)
        uint16x8_t mag = vaddq_u16(vreinterpretq_u16_s16(ax), vreinterpretq_u16_s16(ay));

        // clamp to 255: mag8 = min(mag,255)
        uint16x8_t mag_clamped = vminq_u16(mag, vdupq_n_u16(255));
        return vmovn_u16(mag_clamped);
      };

      uint8x8_t magL = sobel8(p00l,p01l,p02l, p10l,p12l, p20l,p21l,p22l);
      uint8x8_t magH = sobel8(p00h,p01h,p02h, p10h,p12h, p20h,p21h,p22h);
      uint8x16_t mag = vcombine_u8(magL, magH);

      if (invert) {
        mag = vsubq_u8(vdupq_n_u8(255), mag);
      }

      // Store 16 pixels as XRGB with alpha behavior matching your original.
      // For edgesOnly: if mag < threshold OR edge_a==0 => write 0.
      // Else write (edge_a<<24 | mag gray).
      if (mode == filter_cfg::SOBEL_EDGES_ONLY) {
        if (edge_a == 0) {
          // all transparent
          for (int i=0;i<16;i++) dst[x + (uint32_t)i] = 0x00000000u;
        } else {
          uint8x16_t th = vdupq_n_u8(threshold);
          // mask = (mag >= th)
          uint8x16_t m = vcgeq_u8(mag, th);

          // We need scalar store because packing 32-bit pixels with per-lane conditional alpha
          // is messy and usually not the bottleneck compared to Sobel math.
          alignas(16) uint8_t mag_arr[16];
          alignas(16) uint8_t m_arr[16];
          vst1q_u8(mag_arr, mag);
          vst1q_u8(m_arr, m);
          for (int i=0;i<16;i++) {
            if (m_arr[i]) {
              uint8_t v = mag_arr[i];
              dst[x + (uint32_t)i] = ((uint32_t)edge_a<<24) | ((uint32_t)v<<16) | ((uint32_t)v<<8) | (uint32_t)v;
            } else {
              dst[x + (uint32_t)i] = 0x00000000u;
            }
          }
        }
      } else {
        // magnitude mode: opaque grayscale
        alignas(16) uint8_t mag_arr[16];
        vst1q_u8(mag_arr, mag);
        for (int i=0;i<16;i++) {
          uint8_t v = mag_arr[i];
          dst[x + (uint32_t)i] = 0xFF000000u | ((uint32_t)v<<16) | ((uint32_t)v<<8) | (uint32_t)v;
        }
      }
    }
#endif

    // Scalar tail for remaining interior pixels
    for (; x < w-1; x++) {
      int p00 = r0[x-1], p01 = r0[x], p02 = r0[x+1];
      int p10 = r1[x-1],              p12 = r1[x+1];
      int p20 = r2[x-1], p21 = r2[x], p22 = r2[x+1];

      int gx = (-p00 + p02) + (-2*p10 + 2*p12) + (-p20 + p22);
      int gy = (-p00 -2*p01 -p02) + (p20 +2*p21 + p22);

      int mag = abs(gx) + abs(gy);
      if (mag > 255) mag = 255;
      uint8_t m = (uint8_t)mag;
      if (invert) m = (uint8_t)(255 - m);

      if (mode == filter_cfg::SOBEL_EDGES_ONLY) {
        if (m < threshold || edge_a == 0) dst[x] = 0x00000000u;
        else dst[x] = ((uint32_t)edge_a<<24) | ((uint32_t)m<<16) | ((uint32_t)m<<8) | (uint32_t)m;
      } else {
        dst[x] = 0xFF000000u | ((uint32_t)m<<16) | ((uint32_t)m<<8) | (uint32_t)m;
      }
    }
  }

  buf.px.swap(out);
}
/*static void filter_apply_denoise_box(layer_buf &buf, int radius, float strength) {
  radius = std::max(1, std::min(radius, 8));
  strength = std::clamp(strength, 0.0f, 1.0f);
  if (strength <= 0.0f) return;

  const uint32_t w = buf.w, h = buf.h;
  if (w == 0 || h == 0) return;

  // Two-pass box blur (separable): horizontal then vertical, then blend with original.
  std::vector<uint32_t> tmp(buf.px.size());
  std::vector<uint32_t> out(buf.px.size());

  auto idx = [&](uint32_t x, uint32_t y) -> size_t { return (size_t)y * (size_t)w + (size_t)x; };

  // Horizontal pass
  for (uint32_t y = 0; y < h; y++) {
    int sumR=0,sumG=0,sumB=0;
    // init window at x=0
    for (int dx = -radius; dx <= radius; dx++) {
      int xx = std::max(0, std::min((int)w - 1, dx));
      uint32_t p = buf.px[idx((uint32_t)xx, y)];
      sumR += (p >> 16) & 0xFF;
      sumG += (p >> 8) & 0xFF;
      sumB += p & 0xFF;
    }
    int win = radius * 2 + 1;
    for (uint32_t x = 0; x < w; x++) {
      uint8_t r = (uint8_t)(sumR / win);
      uint8_t g = (uint8_t)(sumG / win);
      uint8_t b = (uint8_t)(sumB / win);
      tmp[idx(x,y)] = 0xFF000000u | ((uint32_t)r<<16) | ((uint32_t)g<<8) | (uint32_t)b;

      int x_out = (int)x - radius;
      int x_in  = (int)x + radius + 1;
      x_out = std::max(0, std::min((int)w - 1, x_out));
      x_in  = std::max(0, std::min((int)w - 1, x_in));
      uint32_t p_out = buf.px[idx((uint32_t)x_out, y)];
      uint32_t p_in  = buf.px[idx((uint32_t)x_in, y)];
      sumR += ((p_in >> 16) & 0xFF) - ((p_out >> 16) & 0xFF);
      sumG += ((p_in >> 8) & 0xFF)  - ((p_out >> 8) & 0xFF);
      sumB += (p_in & 0xFF)         - (p_out & 0xFF);
    }
  }

  // Vertical pass
  for (uint32_t x = 0; x < w; x++) {
    int sumR=0,sumG=0,sumB=0;
    for (int dy = -radius; dy <= radius; dy++) {
      int yy = std::max(0, std::min((int)h - 1, dy));
      uint32_t p = tmp[idx(x, (uint32_t)yy)];
      sumR += (p >> 16) & 0xFF;
      sumG += (p >> 8) & 0xFF;
      sumB += p & 0xFF;
    }
    int win = radius * 2 + 1;
    for (uint32_t y = 0; y < h; y++) {
      uint8_t r = (uint8_t)(sumR / win);
      uint8_t g = (uint8_t)(sumG / win);
      uint8_t b = (uint8_t)(sumB / win);
      out[idx(x,y)] = 0xFF000000u | ((uint32_t)r<<16) | ((uint32_t)g<<8) | (uint32_t)b;

      int y_out = (int)y - radius;
      int y_in  = (int)y + radius + 1;
      y_out = std::max(0, std::min((int)h - 1, y_out));
      y_in  = std::max(0, std::min((int)h - 1, y_in));
      uint32_t p_out = tmp[idx(x, (uint32_t)y_out)];
      uint32_t p_in  = tmp[idx(x, (uint32_t)y_in)];
      sumR += ((p_in >> 16) & 0xFF) - ((p_out >> 16) & 0xFF);
      sumG += ((p_in >> 8) & 0xFF)  - ((p_out >> 8) & 0xFF);
      sumB += (p_in & 0xFF)         - (p_out & 0xFF);
    }
  }

  // Blend original with blurred
  if (strength >= 1.0f) {
    buf.px.swap(out);
    return;
  }
  const uint32_t a = (uint32_t)std::lround(strength * 255.0f);
  for (size_t i = 0; i < buf.px.size(); i++) {
    uint32_t orig = buf.px[i];
    uint32_t blur = out[i];
    uint32_t orr=(orig>>16)&0xFF, og=(orig>>8)&0xFF, ob=orig&0xFF;
    uint32_t br =(blur>>16)&0xFF, bg=(blur>>8)&0xFF, bb=blur&0xFF;
    uint32_t rr = (br*a + orr*(255-a) + 127)/255;
    uint32_t gg = (bg*a + og *(255-a) + 127)/255;
    uint32_t bb2= (bb*a + ob *(255-a) + 127)/255;
    buf.px[i] = 0xFF000000u | (rr<<16) | (gg<<8) | bb2;
  }
}*/
/*
  Drop-in faster denoise for RPi CM4:
  - Luma-only pipeline:
      1) Extract luma (Y) from XRGB8888
      2) 3x3 luma clamp (despeckle: removes salt/pepper/speckle)
      3) Small separable 3-tap blur on luma (reduces grain)
      4) Blend with original using 'strength' and write grayscale XRGB
  - NEON is used for the hot loops (AArch64 or ARMv7+NEON).
  - Keeps config semantics: radius 1..8, strength 0..1. Radius maps to pass count.
*/

static inline uint8_t clamp_u8_scalar(int v) {
  return (uint8_t)((v < 0) ? 0 : (v > 255 ? 255 : v));
}

// Clamp center luma to [localMin, localMax] using a 3x3 neighborhood.
// This is a cheap salt/pepper + speckle remover.
static void luma_clamp3x3_neon(std::vector<uint8_t> &y, uint32_t w, uint32_t h) {
  if (w < 3 || h < 3) return;

  std::vector<uint8_t> out(y.size());

  // Copy borders unchanged
  memcpy(out.data(), y.data(), (size_t)w);                               // top row
  memcpy(out.data() + (size_t)(h-1)*w, y.data() + (size_t)(h-1)*w, (size_t)w); // bottom row
  for (uint32_t yy=1; yy+1<h; yy++) {
    out[(size_t)yy*w + 0]     = y[(size_t)yy*w + 0];
    out[(size_t)yy*w + (w-1)] = y[(size_t)yy*w + (w-1)];
  }

  for (uint32_t yy=1; yy+1<h; yy++) {
    const uint8_t *r0 = &y[(size_t)(yy-1)*w];
    const uint8_t *r1 = &y[(size_t)yy*w];
    const uint8_t *r2 = &y[(size_t)(yy+1)*w];
    uint8_t *dst = &out[(size_t)yy*w];

    uint32_t x = 1;

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    // Process 16 pixels at a time for x in [1 .. w-2]
    for (; x + 16 <= w-1; x += 16) {
      // Load 3 rows, with -1/0/+1 shifted windows
      uint8x16_t a0 = vld1q_u8(r0 + x - 1);
      uint8x16_t a1 = vld1q_u8(r0 + x);
      uint8x16_t a2 = vld1q_u8(r0 + x + 1);

      uint8x16_t b0 = vld1q_u8(r1 + x - 1);
      uint8x16_t b1 = vld1q_u8(r1 + x);
      uint8x16_t b2 = vld1q_u8(r1 + x + 1);

      uint8x16_t c0 = vld1q_u8(r2 + x - 1);
      uint8x16_t c1 = vld1q_u8(r2 + x);
      uint8x16_t c2 = vld1q_u8(r2 + x + 1);

      // min over 9 values
      uint8x16_t mn = vminq_u8(a0, a1);
      mn = vminq_u8(mn, a2);
      mn = vminq_u8(mn, b0);
      mn = vminq_u8(mn, b1);
      mn = vminq_u8(mn, b2);
      mn = vminq_u8(mn, c0);
      mn = vminq_u8(mn, c1);
      mn = vminq_u8(mn, c2);

      // max over 9 values
      uint8x16_t mx = vmaxq_u8(a0, a1);
      mx = vmaxq_u8(mx, a2);
      mx = vmaxq_u8(mx, b0);
      mx = vmaxq_u8(mx, b1);
      mx = vmaxq_u8(mx, b2);
      mx = vmaxq_u8(mx, c0);
      mx = vmaxq_u8(mx, c1);
      mx = vmaxq_u8(mx, c2);

      // clamp center (b1) into [mn,mx]
      uint8x16_t v = b1;
      v = vmaxq_u8(v, mn);
      v = vminq_u8(v, mx);

      vst1q_u8(dst + x, v);
    }
#endif

    // Scalar tail for remaining interior pixels
    for (; x < w-1; x++) {
      uint8_t v = r1[x];
      uint8_t mn = 255, mx = 0;
      for (int dy=-1; dy<=1; dy++) {
        const uint8_t *rr = (dy==-1) ? r0 : (dy==0 ? r1 : r2);
        for (int dx=-1; dx<=1; dx++) {
          uint8_t t = rr[x + dx];
          if (t < mn) mn = t;
          if (t > mx) mx = t;
        }
      }
      if (v < mn) v = mn;
      if (v > mx) v = mx;
      dst[x] = v;
    }
  }

  y.swap(out);
}

// 1D 3-tap blur: (1*prev + 2*cur + 1*next) >> 2
// Horizontal pass (NEON), edges copy-through.
static void blur3_h_neon(const std::vector<uint8_t> &in, std::vector<uint8_t> &out, uint32_t w, uint32_t h) {
  out.resize(in.size());
  if (w == 0 || h == 0) return;

  for (uint32_t y=0; y<h; y++) {
    const uint8_t *s = &in[(size_t)y*w];
    uint8_t *d = &out[(size_t)y*w];

    // edges
    d[0] = s[0];
    if (w > 1) d[w-1] = s[w-1];

    uint32_t x = 1;

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    for (; x + 16 <= w-1; x += 16) {
      uint8x16_t a = vld1q_u8(s + x - 1);
      uint8x16_t b = vld1q_u8(s + x);
      uint8x16_t c = vld1q_u8(s + x + 1);

      // widen to u16
      uint16x8_t a0 = vmovl_u8(vget_low_u8(a));
      uint16x8_t b0 = vmovl_u8(vget_low_u8(b));
      uint16x8_t c0 = vmovl_u8(vget_low_u8(c));
      uint16x8_t a1 = vmovl_u8(vget_high_u8(a));
      uint16x8_t b1 = vmovl_u8(vget_high_u8(b));
      uint16x8_t c1 = vmovl_u8(vget_high_u8(c));

      // (a + 2*b + c) >> 2
      uint16x8_t s0 = vaddq_u16(vaddq_u16(a0, c0), vshlq_n_u16(b0, 1));
      uint16x8_t s1 = vaddq_u16(vaddq_u16(a1, c1), vshlq_n_u16(b1, 1));
      uint8x8_t o0 = vshrn_n_u16(s0, 2);
      uint8x8_t o1 = vshrn_n_u16(s1, 2);
      vst1q_u8(d + x, vcombine_u8(o0, o1));
    }
#endif

    for (; x < w-1; x++) {
      int v = (int)s[x-1] + 2*(int)s[x] + (int)s[x+1];
      d[x] = (uint8_t)(v >> 2);
    }
  }
}

// Vertical blur with same kernel.
static void blur3_v_neon(const std::vector<uint8_t> &in, std::vector<uint8_t> &out, uint32_t w, uint32_t h) {
  out.resize(in.size());
  if (w == 0 || h == 0) return;
  if (h == 1) { out = in; return; }

  // top/bottom copy
  memcpy(out.data(), in.data(), (size_t)w);
  memcpy(out.data() + (size_t)(h-1)*w, in.data() + (size_t)(h-1)*w, (size_t)w);

  for (uint32_t y=1; y+1<h; y++) {
    const uint8_t *r0 = &in[(size_t)(y-1)*w];
    const uint8_t *r1 = &in[(size_t)y*w];
    const uint8_t *r2 = &in[(size_t)(y+1)*w];
    uint8_t *d = &out[(size_t)y*w];

    uint32_t x = 0;

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    for (; x + 16 <= w; x += 16) {
      uint8x16_t a = vld1q_u8(r0 + x);
      uint8x16_t b = vld1q_u8(r1 + x);
      uint8x16_t c = vld1q_u8(r2 + x);

      uint16x8_t a0 = vmovl_u8(vget_low_u8(a));
      uint16x8_t b0 = vmovl_u8(vget_low_u8(b));
      uint16x8_t c0 = vmovl_u8(vget_low_u8(c));
      uint16x8_t a1 = vmovl_u8(vget_high_u8(a));
      uint16x8_t b1 = vmovl_u8(vget_high_u8(b));
      uint16x8_t c1 = vmovl_u8(vget_high_u8(c));

      uint16x8_t s0 = vaddq_u16(vaddq_u16(a0, c0), vshlq_n_u16(b0, 1));
      uint16x8_t s1 = vaddq_u16(vaddq_u16(a1, c1), vshlq_n_u16(b1, 1));
      uint8x8_t o0 = vshrn_n_u16(s0, 2);
      uint8x8_t o1 = vshrn_n_u16(s1, 2);
      vst1q_u8(d + x, vcombine_u8(o0, o1));
    }
#endif

    for (; x < w; x++) {
      int v = (int)r0[x] + 2*(int)r1[x] + (int)r2[x];
      d[x] = (uint8_t)(v >> 2);
    }
  }
}

static void filter_apply_denoise_box(layer_buf &buf, int radius, float strength) {
  radius = std::max(1, std::min(radius, 8));
  strength = std::clamp(strength, 0.0f, 1.0f);
  if (strength <= 0.0f) return;

  const uint32_t w = buf.w, h = buf.h;
  if (w == 0 || h == 0) return;
  if (buf.px.empty()) return;

  // Map "radius" to small pass counts to keep it fast and deterministic.
  // - radius 1-2: 1 blur pass
  // - radius 3-5: 2 blur passes
  // - radius 6-8: 3 blur passes
  int blur_passes = (radius <= 2) ? 1 : (radius <= 5 ? 2 : 3);

  // 1) Extract luma
  std::vector<uint8_t> y(buf.px.size());
  for (size_t i=0; i<buf.px.size(); i++) {
    y[i] = luma_u8_from_xrgb(buf.px[i]);
  }

  // 2) Despeckle / salt-pepper: clamp to local range
  luma_clamp3x3_neon(y, w, h);

  // 3) Small blur passes
  std::vector<uint8_t> tmp;
  std::vector<uint8_t> tmp2;
  for (int p=0; p<blur_passes; p++) {
    blur3_h_neon(y, tmp, w, h);
    blur3_v_neon(tmp, tmp2, w, h);
    y.swap(tmp2);
  }

  // 4) Blend with original using strength; write grayscale
  if (strength >= 1.0f) {
    for (size_t i=0; i<buf.px.size(); i++) {
      uint8_t v = y[i];
      buf.px[i] = 0xFF000000u | ((uint32_t)v<<16) | ((uint32_t)v<<8) | (uint32_t)v;
    }
    return;
  }

  const uint32_t a = (uint32_t)std::lround(strength * 255.0f);
  for (size_t i=0; i<buf.px.size(); i++) {
    uint32_t orig = buf.px[i];
    uint32_t orr = (orig >> 16) & 0xFF;
    uint32_t og  = (orig >> 8) & 0xFF;
    uint32_t ob  = (orig) & 0xFF;

    uint32_t v = y[i];
    // Blend each channel toward luma value (keeps "strength" meaning consistent with your other filters)
    uint32_t rr = (v*a + orr*(255-a) + 127)/255;
    uint32_t gg = (v*a + og *(255-a) + 127)/255;
    uint32_t bb = (v*a + ob *(255-a) + 127)/255;

    buf.px[i] = 0xFF000000u | (rr<<16) | (gg<<8) | bb;
  }
}
static inline bool chan_match(int mn, int mx, uint8_t v) {
  // wildcard
  if (mn == -1 && mx == -1) return true;
  return (v >= (uint8_t)mn && v <= (uint8_t)mx);
}
/*static void filter_apply_rgb_map(layer_buf &buf,
                                 int rmin, int rmax, uint8_t rout,
                                 int gmin, int gmax, uint8_t gout,
                                 int bmin, int bmax, uint8_t bout,
                                 int amin, int amax, uint8_t aout) {
  if (buf.w == 0 || buf.h == 0) return;

  for (size_t i = 0; i < buf.px.size(); i++) {
    uint32_t p = buf.px[i];
    uint8_t pa = (uint8_t)((p >> 24) & 0xFF);
    uint8_t r = (uint8_t)((p >> 16) & 0xFF);
    uint8_t g = (uint8_t)((p >> 8) & 0xFF);
    uint8_t b = (uint8_t)(p & 0xFF);

    if (chan_match(rmin, rmax, r) &&
        chan_match(gmin, gmax, g) &&
        chan_match(bmin, bmax, b) &&
        chan_match(amin, amax, pa)) {
      buf.px[i] = ((uint32_t)aout << 24) | ((uint32_t)rout << 16) | ((uint32_t)gout << 8) | (uint32_t)bout;
    }
  }
}*/
static inline bool chan_is_wildcard(int mn, int mx) { return (mn == -1 && mx == -1); }

// NEON accelerated RGBA range -> RGBA value mapping.
// Semantics match your existing rgbMap:
// - If mn/mx are both -1 => wildcard (channel always matches)
// - Otherwise inclusive range match: mn <= channel <= mx
// - If all channels match => pixel becomes (aout,rout,gout,bout) packed as A in top byte
static void filter_apply_rgb_map(layer_buf &buf,
                                 int rmin, int rmax, uint8_t rout,
                                 int gmin, int gmax, uint8_t gout,
                                 int bmin, int bmax, uint8_t bout,
                                 int amin, int amax, uint8_t aout) {
  if (buf.w == 0 || buf.h == 0) return;
  if (buf.px.empty()) return;

  const bool r_wc = chan_is_wildcard(rmin, rmax);
  const bool g_wc = chan_is_wildcard(gmin, gmax);
  const bool b_wc = chan_is_wildcard(bmin, bmax);
  const bool a_wc = chan_is_wildcard(amin, amax);

  // Clamp non-wildcard mins/maxes to [0..255] to avoid UB in comparisons
  auto clamp_mm = [](int &mn, int &mx) {
    if (mn == -1 && mx == -1) return;
    mn = std::max(0, std::min(255, mn));
    mx = std::max(0, std::min(255, mx));
    if (mx < mn) std::swap(mx, mn);
  };
  clamp_mm(rmin, rmax);
  clamp_mm(gmin, gmax);
  clamp_mm(bmin, bmax);
  clamp_mm(amin, amax);

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
  const uint32x4_t vout0 = vdupq_n_u32(((uint32_t)aout << 24) | ((uint32_t)rout << 16) | ((uint32_t)gout << 8) | (uint32_t)bout);

  size_t i = 0;
  const size_t n = buf.px.size();

  // Process 16 pixels (64 bytes) per iteration
  for (; i + 16 <= n; i += 16) {
    // Load 16 packed pixels as 4x uint32x4
    const uint32_t *src = &buf.px[i];
    uint32x4_t p0 = vld1q_u32(src + 0);
    uint32x4_t p1 = vld1q_u32(src + 4);
    uint32x4_t p2 = vld1q_u32(src + 8);
    uint32x4_t p3 = vld1q_u32(src + 12);

    // Build per-4-pixel 32-bit lane masks directly (clean vbslq_u32 use).
    // Extract 32-bit lanes for each channel by shifting and masking.
    const uint32x4_t vff = vdupq_n_u32(0xFFFFFFFFu);

    auto chan_mask_u32 = [&](uint32x4_t chan, int mn, int mx, bool wc) -> uint32x4_t {
      if (wc) return vff;
      uint32x4_t vmn = vdupq_n_u32((uint32_t)mn);
      uint32x4_t vmx = vdupq_n_u32((uint32_t)mx);
      uint32x4_t ge = vcgeq_u32(chan, vmn);
      uint32x4_t le = vcleq_u32(chan, vmx);
      return vandq_u32(ge, le); // 0xFFFFFFFF where in range
    };

    auto unpack_chan = [&](uint32x4_t p, int shift) -> uint32x4_t {
      if (shift == 0) return vandq_u32(p, vdupq_n_u32(0xFFu));
      return vandq_u32(vshrq_n_u32(p, shift), vdupq_n_u32(0xFFu));
    };

    // r/g/b/a values as u32 lanes (0..255)
    uint32x4_t r0 = unpack_chan(p0, 16);
    uint32x4_t g0 = unpack_chan(p0, 8);
    uint32x4_t b0 = unpack_chan(p0, 0);
    uint32x4_t a0 = unpack_chan(p0, 24);

    uint32x4_t r1 = unpack_chan(p1, 16);
    uint32x4_t g1 = unpack_chan(p1, 8);
    uint32x4_t b1 = unpack_chan(p1, 0);
    uint32x4_t a1 = unpack_chan(p1, 24);

    uint32x4_t r2 = unpack_chan(p2, 16);
    uint32x4_t g2 = unpack_chan(p2, 8);
    uint32x4_t b2 = unpack_chan(p2, 0);
    uint32x4_t a2 = unpack_chan(p2, 24);

    uint32x4_t r3 = unpack_chan(p3, 16);
    uint32x4_t g3 = unpack_chan(p3, 8);
    uint32x4_t b3 = unpack_chan(p3, 0);
    uint32x4_t a3 = unpack_chan(p3, 24);

    // per-lane masks
    uint32x4_t mr0 = chan_mask_u32(r0, rmin, rmax, r_wc);
    uint32x4_t mg0 = chan_mask_u32(g0, gmin, gmax, g_wc);
    uint32x4_t mb0 = chan_mask_u32(b0, bmin, bmax, b_wc);
    uint32x4_t ma0 = chan_mask_u32(a0, amin, amax, a_wc);
    uint32x4_t m0  = vandq_u32(vandq_u32(mr0, mg0), vandq_u32(mb0, ma0));

    uint32x4_t mr1 = chan_mask_u32(r1, rmin, rmax, r_wc);
    uint32x4_t mg1 = chan_mask_u32(g1, gmin, gmax, g_wc);
    uint32x4_t mb1 = chan_mask_u32(b1, bmin, bmax, b_wc);
    uint32x4_t ma1 = chan_mask_u32(a1, amin, amax, a_wc);
    uint32x4_t m1  = vandq_u32(vandq_u32(mr1, mg1), vandq_u32(mb1, ma1));

    uint32x4_t mr2 = chan_mask_u32(r2, rmin, rmax, r_wc);
    uint32x4_t mg2 = chan_mask_u32(g2, gmin, gmax, g_wc);
    uint32x4_t mb2 = chan_mask_u32(b2, bmin, bmax, b_wc);
    uint32x4_t ma2 = chan_mask_u32(a2, amin, amax, a_wc);
    uint32x4_t m2  = vandq_u32(vandq_u32(mr2, mg2), vandq_u32(mb2, ma2));

    uint32x4_t mr3 = chan_mask_u32(r3, rmin, rmax, r_wc);
    uint32x4_t mg3 = chan_mask_u32(g3, gmin, gmax, g_wc);
    uint32x4_t mb3 = chan_mask_u32(b3, bmin, bmax, b_wc);
    uint32x4_t ma3 = chan_mask_u32(a3, amin, amax, a_wc);
    uint32x4_t m3  = vandq_u32(vandq_u32(mr3, mg3), vandq_u32(mb3, ma3));

    // Normalize masks to all-bits set or zero for vbsl.
    // Any nonzero becomes 0xFFFFFFFF by negating compare against 0.
    auto norm_mask = [](uint32x4_t mm) -> uint32x4_t {
      // mm currently has 0x00..00FF style values; compare !=0 -> 0xFFFFFFFF
      return vcgtq_u32(mm, vdupq_n_u32(0));
    };
    m0 = norm_mask(m0);
    m1 = norm_mask(m1);
    m2 = norm_mask(m2);
    m3 = norm_mask(m3);

    // Select output pixel or keep original
    p0 = vbslq_u32(m0, vout0, p0);
    p1 = vbslq_u32(m1, vout0, p1);
    p2 = vbslq_u32(m2, vout0, p2);
    p3 = vbslq_u32(m3, vout0, p3);

    vst1q_u32((uint32_t*)src + 0, p0);
    vst1q_u32((uint32_t*)src + 4, p1);
    vst1q_u32((uint32_t*)src + 8, p2);
    vst1q_u32((uint32_t*)src + 12, p3);
  }

  // Scalar tail
  for (; i < n; i++) {
    uint32_t p = buf.px[i];
    uint8_t pr = (uint8_t)((p >> 16) & 0xFF);
    uint8_t pg = (uint8_t)((p >> 8) & 0xFF);
    uint8_t pb = (uint8_t)(p & 0xFF);
    uint8_t pa = (uint8_t)((p >> 24) & 0xFF);

    auto match = [&](int mn, int mx, uint8_t v) -> bool {
      if (mn == -1 && mx == -1) return true;
      return (v >= (uint8_t)mn && v <= (uint8_t)mx);
    };

    if (match(rmin, rmax, pr) && match(gmin, gmax, pg) && match(bmin, bmax, pb) && match(amin, amax, pa)) {
      buf.px[i] = ((uint32_t)aout << 24) | ((uint32_t)rout << 16) | ((uint32_t)gout << 8) | (uint32_t)bout;
    }
  }
#else
  // No NEON: scalar fallback (should match your existing behavior)
  auto match = [&](int mn, int mx, uint8_t v) -> bool {
    if (mn == -1 && mx == -1) return true;
    return (v >= (uint8_t)mn && v <= (uint8_t)mx);
  };

  for (size_t i=0; i<buf.px.size(); i++) {
    uint32_t p = buf.px[i];
    uint8_t pr = (uint8_t)((p >> 16) & 0xFF);
    uint8_t pg = (uint8_t)((p >> 8) & 0xFF);
    uint8_t pb = (uint8_t)(p & 0xFF);
    uint8_t pa = (uint8_t)((p >> 24) & 0xFF);

    if (match(rmin, rmax, pr) && match(gmin, gmax, pg) && match(bmin, bmax, pb) && match(amin, amax, pa)) {
      buf.px[i] = ((uint32_t)aout << 24) | ((uint32_t)rout << 16) | ((uint32_t)gout << 8) | (uint32_t)bout;
    }
  }
#endif
}
static inline bool is_key_black(uint8_t r, uint8_t g, uint8_t b, int th) {
  // "close to black": all channels <= threshold
  return (r <= th) && (g <= th) && (b <= th);
}
static inline bool is_key_white(uint8_t r, uint8_t g, uint8_t b, int th) {
  // "close to white": all channels >= 255-th
  const int lo = 255 - th;
  return (r >= lo) && (g >= lo) && (b >= lo);
}

static void filter_apply_rgb_key_alpha(layer_buf &buf,
                                       filter_cfg::key_mode_t mode,
                                       int threshold,
                                       uint8_t keyA,
                                       uint8_t keepA,
                                       bool setRgb,
                                       uint8_t outR, uint8_t outG, uint8_t outB) {
  if (buf.w == 0 || buf.h == 0) return;
  threshold = std::max(0, std::min(threshold, 255));

  for (size_t i = 0; i < buf.px.size(); i++) {
    uint32_t p = buf.px[i];
    uint8_t r = (uint8_t)((p >> 16) & 0xFF);
    uint8_t g = (uint8_t)((p >> 8) & 0xFF);
    uint8_t b = (uint8_t)(p & 0xFF);

    bool keyed = (mode == filter_cfg::KEY_WHITE)
      ? is_key_white(r, g, b, threshold)
      : is_key_black(r, g, b, threshold);

    uint8_t a = keyed ? keyA : keepA;

    if (setRgb) {
      r = outR; g = outG; b = outB;
    }
    buf.px[i] = ((uint32_t)a << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
  }
}
static void apply_filter_chain(layer_buf &buf, const std::vector<filter_cfg> &filters) {
  for (const auto &f : filters) {
    if (f.id == FILTER_MONO) {
      filter_apply_mono(buf, f.mono_strength);
    } else if (f.id == FILTER_SOBEL) {
      filter_apply_sobel(buf, f.sobel_mode, f.sobel_threshold, f.sobel_alpha, f.sobel_invert);
    } else if (f.id == FILTER_DENOISE) {
      filter_apply_denoise_box(buf, f.denoise_radius, f.denoise_strength);
    } else if (f.id == FILTER_RGB_MAP) {
      filter_apply_rgb_map(buf,
                           f.r_min, f.r_max, f.r_out,
                           f.g_min, f.g_max, f.g_out,
                           f.b_min, f.b_max, f.b_out,
                           f.a_min, f.a_max, f.a_out);
    } else if (f.id == FILTER_RGB_KEY_ALPHA) {
      filter_apply_rgb_key_alpha(buf, f.key_mode, f.key_threshold, f.key_alpha, f.keep_alpha, f.set_rgb, f.out_r, f.out_g, f.out_b);
    }
  }
}

// ---------------- Source capture: normalize to XRGB8888 ----------------
struct latest_frame {
  const uint32_t *ptr_xrgb=nullptr;
  uint32_t stride_bytes=0;
  uint32_t w=0,h=0;
  uint32_t buf_index=0;
  bool valid=false;
};
struct source_capture {
  std::string dev;
  bool is_tc358743=false;
  bool have_held_buf=false;
  struct v4l2_buffer held_buf{};
  int vfd=-1;
  cap_buf *cbufs=nullptr;
  uint32_t cbuf_count=0;
  uint32_t in_w=0,in_h=0,in_stride=0,in_pixfmt=0;
  uint32_t dv_w=0,dv_h=0;
  uint64_t last_dv_check_ms=0;
  uint64_t dv_check_interval_ms=500;
  latest_frame latest{};
  std::mutex latest_mtx;
  std::atomic<uint64_t> latest_seq{0};
  std::vector<uint8_t> conv_xrgb;
  uint32_t conv_stride_bytes=0;
  source_probe last_probe{};
  std::mutex probe_mtx;

  // Prefer UYVY/YUYV for TC358743 if supported, fallback to RGB24.
  // This avoids forcing RGB24 (3 bytes/pixel) which can be more expensive for some devices/drivers.
  bool init_tc358743(const persisted_config &cfg, const std::string &dev_path) {
    is_tc358743 = true;
    dev = dev_path;
    fprintf(stderr, "[source] init TC358743 %s\n", dev.c_str());
    vfd = v4l2_open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (vfd < 0) { perror("v4l2_open"); return false; }
    if (!cfg.edid_path.empty()) {
      (void)v4l2_set_edid_if_requested(vfd, cfg.edid_path.c_str());
      usleep(1500*1000);
    }
    uint64_t pixelclock=0;
    dv_w=dv_h=0;
    if (tc358743_query_and_set_dv_timings(vfd, &dv_w, &dv_h, &pixelclock, true) != 0) {
      fprintf(stderr, "[source] %s: no DV timings (no signal?)\n", dev.c_str());
      return false;
    }

    // ---- OPTIMIZATION CHANGE: prefer 4:2:2 formats ----
    // Try UYVY then YUYV, then fallback to RGB24.
    if (v4l2_set_fmt_capture(vfd, V4L2_PIX_FMT_UYVY, 0, 0, &in_w, &in_h, &in_stride) == 0) {
      in_pixfmt = V4L2_PIX_FMT_UYVY;
      fprintf(stderr, "[source] %s: using UYVY %ux%u stride=%u\n", dev.c_str(), in_w, in_h, in_stride);
    } else if (v4l2_set_fmt_capture(vfd, V4L2_PIX_FMT_YUYV, 0, 0, &in_w, &in_h, &in_stride) == 0) {
      in_pixfmt = V4L2_PIX_FMT_YUYV;
      fprintf(stderr, "[source] %s: using YUYV %ux%u stride=%u\n", dev.c_str(), in_w, in_h, in_stride);
    } else {
      if (tc358743_set_pixfmt_rgb24(vfd, &in_w, &in_h, &in_stride) != 0) return false;
      in_pixfmt = V4L2_PIX_FMT_RGB24;
      fprintf(stderr, "[source] %s: using RGB24 %ux%u stride=%u\n", dev.c_str(), in_w, in_h, in_stride);
    }

    if (v4l2_start_mmap_capture(vfd, &cbufs, &cbuf_count) != 0) return false;
    have_held_buf=false;
    memset(&held_buf, 0, sizeof(held_buf));
    latest_seq.store(0);
    {
      std::lock_guard<std::mutex> lk(probe_mtx);
      last_probe = probe_v4l2_source(dev);
    }
    return true;
  }

  uint32_t pick_best_pixfmt_for_aux(const v4l2_device_caps &caps) {
    // Preference order for aux devices.
    // (MJPG intentionally not supported yet)
    const uint32_t prefer[] = {
      V4L2_PIX_FMT_XRGB32,
      V4L2_PIX_FMT_RGB24,
      V4L2_PIX_FMT_YUYV,
      V4L2_PIX_FMT_UYVY,
    };
    for (uint32_t p : prefer) {
      for (auto &f : caps.formats) if (f.pixfmt == p) return p;
    }
    return 0;
  }

  bool init_aux(const persisted_config &cfg, const aux_source_cfg &aux) {
    is_tc358743 = false;
    dev = aux.dev;
    fprintf(stderr, "[source] init AUX %s\n", dev.c_str());
    vfd = v4l2_open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (vfd < 0) { perror("v4l2_open"); return false; }

    v4l2_device_caps caps = v4l2_query_caps(dev);
    if (!caps.ok) {
      fprintf(stderr, "[source] %s: caps query failed: %s\n", dev.c_str(), caps.err.c_str());
      return false;
    }

    uint32_t chosen_pixfmt = 0;
    uint32_t want_w = 0, want_h = 0;

    if (aux.mode == "manual") {
      chosen_pixfmt = string_to_fourcc(aux.pixfmt);
      want_w = aux.width;
      want_h = aux.height;
      if (!chosen_pixfmt || want_w == 0 || want_h == 0) {
        fprintf(stderr, "[source] %s: invalid manual mode settings\n", dev.c_str());
        return false;
      }
      if (chosen_pixfmt == V4L2_PIX_FMT_MJPEG) {
        fprintf(stderr, "[source] %s: MJPEG not supported yet (choose YUYV/UYVY/RGB24)\n", dev.c_str());
        return false;
      }
    } else {
      chosen_pixfmt = pick_best_pixfmt_for_aux(caps);
      if (!chosen_pixfmt) {
        fprintf(stderr, "[source] %s: no supported pixfmt found (need RGB24/YUYV/UYVY)\n", dev.c_str());
        return false;
      }
      // If caps includes sizes for this pixfmt, pick the first size; else leave 0 to accept default.
      for (auto &f : caps.formats) {
        if (f.pixfmt != chosen_pixfmt) continue;
        if (!f.sizes.empty()) {
          want_w = f.sizes[0].first;
          want_h = f.sizes[0].second;
        }
        break;
      }
    }

    if (v4l2_set_fmt_capture(vfd, chosen_pixfmt, want_w, want_h, &in_w, &in_h, &in_stride) != 0) {
      fprintf(stderr, "[source] %s: VIDIOC_S_FMT failed: %s\n", dev.c_str(), strerror(errno));
      return false;
    }
    in_pixfmt = chosen_pixfmt;
    if (v4l2_start_mmap_capture(vfd, &cbufs, &cbuf_count) != 0) return false;
    have_held_buf=false;
    memset(&held_buf, 0, sizeof(held_buf));
    latest_seq.store(0);
    {
      std::lock_guard<std::mutex> lk(probe_mtx);
      last_probe = probe_v4l2_source(dev);
    }
    return true;
  }

  void shutdown() {
    have_held_buf=false;
    memset(&held_buf, 0, sizeof(held_buf));
    if (vfd>=0) {
      v4l2_stop_and_unmap(vfd, cbufs, cbuf_count);
      cbufs=nullptr; cbuf_count=0;
      v4l2_close(vfd);
      vfd=-1;
    }
    {
      std::lock_guard<std::mutex> lk(latest_mtx);
      latest = {};
      latest.valid=false;
    }
    latest_seq.store(0);
  }

  bool detect_input_change_and_request_reinit(std::string &out_reason) {
    // TC358743 DV timings tracking remains
    if (is_tc358743) {
      uint32_t cur_w=0,cur_h=0,cur_stride=0,cur_pixfmt=0;
      if (v4l2_get_current_fmt(vfd, &cur_w,&cur_h,&cur_stride,&cur_pixfmt)==0) {
        if (cur_w && cur_h && (cur_w!=in_w || cur_h!=in_h || cur_stride!=in_stride)) {
          out_reason = "input resolution changed (VIDIOC_G_FMT) on " + dev;
          return true;
        }
      }
      uint64_t now=monotonic_ms();
      if ((now-last_dv_check_ms) < dv_check_interval_ms) return false;
      last_dv_check_ms = now;
      uint32_t tw=0,th=0; uint64_t pc=0;
      if (tc358743_query_dv_timings(vfd, &tw,&th,&pc)==0) {
        if (tw && th && (tw!=dv_w || th!=dv_h)) {
          out_reason = "input resolution changed (DV timings) on " + dev;
          return true;
        }
      } else {
        out_reason = "DV timings unavailable (signal lost?) on " + dev;
        return true;
      }
    }
    return false;
  }

  bool step_capture_publish_blocking(const persisted_config &cfg, int timeout_ms, bool *got_new_frame) {
    if (got_new_frame) *got_new_frame = false;
    uint64_t before_seq = latest_seq.load(std::memory_order_acquire);

    int prc = v4l2_wait_for_frame(vfd, timeout_ms);
    if (prc < 0) return false;
    if (prc == 0) return true;

    struct v4l2_buffer b;
    int dq=v4l2_dequeue_latest_frame(vfd, &b);
    if (dq==1) return true;
    if (dq<0) return false;

    const uint8_t *src = (const uint8_t*)cbufs[b.index].start;

    // Convert to tight-packed XRGB8888
    conv_stride_bytes = in_w * 4;
    conv_xrgb.resize((size_t)in_w * (size_t)in_h * 4);
    uint32_t *dst = (uint32_t*)conv_xrgb.data();

    if (in_pixfmt == V4L2_PIX_FMT_RGB24) {
      convert_rgb24_to_xrgb8888(src, in_stride, dst, conv_stride_bytes, in_w, in_h, cfg.input_is_bgr);
    } else if (in_pixfmt == V4L2_PIX_FMT_YUYV) {
      convert_yuyv_to_xrgb8888(src, in_stride, dst, conv_stride_bytes, in_w, in_h);
    } else if (in_pixfmt == V4L2_PIX_FMT_UYVY) {
      convert_uyvy_to_xrgb8888(src, in_stride, dst, conv_stride_bytes, in_w, in_h);
    } else if (in_pixfmt == V4L2_PIX_FMT_XRGB32) {
      // If provided as XRGB32 already, just copy line-by-line (respect stride)
      for (uint32_t y=0; y<in_h; y++) {
        memcpy((uint8_t*)dst + (size_t)y*(size_t)conv_stride_bytes,
               src + (uint64_t)y*(uint64_t)in_stride,
               (size_t)in_w*4);
      }
    } else {
      // Unsupported => black
      memset(dst, 0, (size_t)in_w * (size_t)in_h * 4);
    }

    if (have_held_buf) (void)v4l2_queue_frame(vfd, &held_buf);
    held_buf = b;
    have_held_buf = true;

    {
      std::lock_guard<std::mutex> lk(latest_mtx);
      latest.ptr_xrgb = (const uint32_t*)conv_xrgb.data();
      latest.stride_bytes = conv_stride_bytes;
      latest.w=in_w; latest.h=in_h;
      latest.buf_index=b.index;
      latest.valid=true;
    }

    latest_seq.fetch_add(1, std::memory_order_release);
    uint64_t after_seq = latest_seq.load(std::memory_order_acquire);
    if (got_new_frame && after_seq != before_seq) *got_new_frame = true;

    // update probe snapshot periodically (simple open+query)
    static const uint64_t kProbeEveryMs = 1000;
    static thread_local uint64_t last_probe_ms = 0;
    uint64_t now = monotonic_ms();
    if (now - last_probe_ms >= kProbeEveryMs) {
      last_probe_ms = now;
      source_probe p = probe_v4l2_source(dev);
      std::lock_guard<std::mutex> lk(probe_mtx);
      last_probe = p;
    }

    return true;
  }

  latest_frame latest_snapshot() {
    std::lock_guard<std::mutex> lk(latest_mtx);
    return latest;
  }
  source_probe probe_snapshot() {
    std::lock_guard<std::mutex> lk(probe_mtx);
    return last_probe;
  }
};

// ---------------- Runtime status ----------------
struct runtime_info {
  std::mutex mtx;
  uint32_t out_w = 0;
  uint32_t out_h = 0;
  uint32_t vpw = 0;
  uint32_t vph = 0;
  present_rect pr{};
  std::string present_policy = "fit";
  struct source_rt {
    std::string dev;
    bool present=false;
    bool active=false;
    uint32_t dv_w=0, dv_h=0;
    uint64_t pixelclock=0;
    uint32_t fmt_w=0, fmt_h=0;
    uint32_t stride=0;
    uint32_t pixfmt=0;
    std::string err;
  };
  std::vector<source_rt> sources;
  struct layer_rt {
    std::string requested;
    std::string resolved;
    bool active=false;
  };
  std::vector<layer_rt> layerSources;
};
static runtime_info g_rt;

// ---------------- Thread pool compositor (kept from your architecture) ----------------
struct comp_job {
  uint64_t job_id=0;
  struct source_frame {
    std::string dev;
    const uint32_t *src_xrgb=nullptr;
    uint32_t src_stride_bytes=0;
    uint32_t in_w=0,in_h=0;
    bool valid=false;
  };
  std::vector<source_frame> sources;
  uint8_t *dst_xrgb=nullptr;
  uint32_t dst_stride=0;
  uint32_t out_w=0,out_h=0;
  present_rect map_pr{};
  uint32_t vpw=0,vph=0;
  const std::vector<layer_cfg> *layers=nullptr;
  const out_to_vp_lut *lut=nullptr;
  const std::vector<layer_map_cache> *layer_maps=nullptr;
  const std::vector<int> *layer_source_index=nullptr;
};
struct comp_worker { pthread_t th{}; int id=0; int cpu=-1; uint32_t y0=0,y1=0; };
static pthread_mutex_t g_comp_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_comp_cv_job = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  g_comp_cv_done = PTHREAD_COND_INITIALIZER;
static comp_job g_comp_job{};
static bool g_comp_have_job=false;
static bool g_comp_stop=false;
static int g_comp_workers=0;
static int g_comp_done=0;
static uint64_t g_comp_cur_job_id=0;
static comp_worker *g_comp_ws=nullptr;
static void pin_thread_to_cpu(int cpu) {
#ifdef __linux__
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  (void)pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
#else
  (void)cpu;
#endif
}
static inline const comp_job::source_frame* job_find_source(const comp_job &job, int source_index) {
  if (source_index < 0) return nullptr;
  if ((size_t)source_index >= job.sources.size()) return nullptr;
  const auto &sf = job.sources[(size_t)source_index];
  if (!sf.valid || !sf.src_xrgb) return nullptr;
  return &sf;
}
static void composite_video_direct_rows(const comp_job &job,
                                       const layer_map_cache &M,
                                       uint32_t y0, uint32_t y1,
                                       const comp_job::source_frame &sf) {
  const out_to_vp_lut &lut = *job.lut;
  const uint8_t a = M.alpha;
  rect_i32 ob = layer_vp_rect_to_output_bounds(job.map_pr, job.vpw, job.vph, M.dst_x, M.dst_y, M.vp_w, M.vp_h);
  ob = clamp_bounds_to_output_and_band(ob, job.out_w, job.out_h, y0, y1);
  if (rect_empty(ob)) return;
  for (int32_t oy=ob.y0; oy<ob.y1; oy++) {
    int32_t vy = lut.oy_to_vy[(uint32_t)oy];
    if (vy < 0) continue;
    int32_t lvy = vy - M.dst_y;
    if ((uint32_t)lvy >= (uint32_t)M.vp_h) continue;
    uint32_t syi = M.sr.y + (uint32_t)M.y_map[(size_t)lvy];
    if (syi >= sf.in_h) continue;
    uint32_t *drow = (uint32_t*)(job.dst_xrgb + (uint64_t)oy * job.dst_stride);
    const uint32_t *srow = (const uint32_t*)((const uint8_t*)sf.src_xrgb + (uint64_t)syi * sf.src_stride_bytes);
    for (int32_t ox=ob.x0; ox<ob.x1; ox++) {
      int32_t vx = lut.ox_to_vx[(uint32_t)ox];
      if (vx < 0) continue;
      int32_t lvx = vx - M.dst_x;
      if ((uint32_t)lvx >= (uint32_t)M.vp_w) continue;
      uint32_t sxi = M.sr.x + (uint32_t)M.x_map[(size_t)lvx];
      if (sxi >= sf.in_w) continue;
      uint32_t over = srow[sxi];
      if (a == 255 && M.inv == INV_NONE) drow[(uint32_t)ox] = over;
      else {
        if (M.inv == INV_LOWER) over = invert_xrgb8888(drow[(uint32_t)ox]);
        drow[(uint32_t)ox] = blend_over_xrgb(drow[(uint32_t)ox], over, a);
      }
    }
  }
}
static void composite_video_filtered_rows(const comp_job &job,
                                         const layer_cfg &L,
                                         const layer_map_cache &M,
                                         uint32_t y0, uint32_t y1,
                                         const comp_job::source_frame &sf) {
  const out_to_vp_lut &lut = *job.lut;
  rect_i32 ob = layer_vp_rect_to_output_bounds(job.map_pr, job.vpw, job.vph, M.dst_x, M.dst_y, M.vp_w, M.vp_h);
  rect_i32 ob_full = clamp_bounds_to_output_and_band(ob, job.out_w, job.out_h, 0, job.out_h);
  if (rect_empty(ob_full)) return;

  layer_buf buf;
  buf.resize((uint32_t)M.vp_w, (uint32_t)M.vp_h);
  for (int32_t lvy=0; lvy<M.vp_h; lvy++) {
    uint32_t syi = M.sr.y + (uint32_t)M.y_map[(size_t)lvy];
    uint32_t *dst = &buf.px[(size_t)lvy*(size_t)buf.w];
    if (syi >= sf.in_h) { std::fill(dst, dst + buf.w, 0xFF000000u); continue; }
    const uint32_t *srow = (const uint32_t*)((const uint8_t*)sf.src_xrgb + (uint64_t)syi * sf.src_stride_bytes);
    for (int32_t lvx=0; lvx<M.vp_w; lvx++) {
      uint32_t sxi = M.sr.x + (uint32_t)M.x_map[(size_t)lvx];
      dst[(size_t)lvx] = (sxi < sf.in_w) ? srow[sxi] : 0xFF000000u;
    }
  }

  apply_filter_chain(buf, L.filters);

  rect_i32 ob_band = clamp_bounds_to_output_and_band(ob, job.out_w, job.out_h, y0, y1);
  if (rect_empty(ob_band)) return;
  const uint8_t layer_a = M.alpha;
  if (layer_a == 0) return;

  for (int32_t oy=ob_band.y0; oy<ob_band.y1; oy++) {
    int32_t vy = lut.oy_to_vy[(uint32_t)oy];
    if (vy < 0) continue;
    int32_t lvy = vy - M.dst_y;
    if ((uint32_t)lvy >= (uint32_t)M.vp_h) continue;
    uint32_t *drow = (uint32_t*)(job.dst_xrgb + (uint64_t)oy * job.dst_stride);
    const uint32_t *srcRow = &buf.px[(size_t)lvy*(size_t)buf.w];
    for (int32_t ox=ob_band.x0; ox<ob_band.x1; ox++) {
      int32_t vx = lut.ox_to_vx[(uint32_t)ox];
      if (vx < 0) continue;
      int32_t lvx = vx - M.dst_x;
      if ((uint32_t)lvx >= (uint32_t)M.vp_w) continue;
      uint32_t over = srcRow[(size_t)lvx];
      uint8_t over_a = (uint8_t)((over >> 24) & 0xFF);
      if (over_a == 0) continue;
      uint8_t final_a = (uint8_t)((uint32_t)over_a * (uint32_t)layer_a / 255u);
      uint32_t over_xrgb = 0xFF000000u | (over & 0x00FFFFFFu);
      drow[(uint32_t)ox] = blend_over_xrgb(drow[(uint32_t)ox], over_xrgb, final_a);
    }
  }
}
static void composite_layers_rows(const comp_job &job, uint32_t y0, uint32_t y1) {
  const auto &layers = *job.layers;
  const auto &maps = *job.layer_maps;
  clear_xrgb_rows_fast(job.dst_xrgb, job.dst_stride, job.out_w, y0, y1);
  for (size_t li=0; li<layers.size(); li++) {
    const layer_cfg &L = layers[li];
    if (!L.enabled) continue;
    if (L.type != LAYER_VIDEO) continue;
    int src_idx = -1;
    if (job.layer_source_index && li < job.layer_source_index->size()) src_idx = (*job.layer_source_index)[li];
    const comp_job::source_frame *sf = job_find_source(job, src_idx);
    if (!sf) continue;
    const layer_map_cache &M = maps[li];
    if (!M.enabled) continue;
    if (!L.filters.empty()) composite_video_filtered_rows(job, L, M, y0, y1, *sf);
    else composite_video_direct_rows(job, M, y0, y1, *sf);
  }
}
static void *comp_worker_main(void *arg) {
  comp_worker *w = (comp_worker*)arg;
  if (w->cpu >= 0) pin_thread_to_cpu(w->cpu);
  uint64_t last_seen_job_id = 0;
  for (;;) {
    pthread_mutex_lock(&g_comp_mtx);
    while ((!g_comp_have_job || g_comp_cur_job_id == last_seen_job_id) && !g_comp_stop) {
      pthread_cond_wait(&g_comp_cv_job, &g_comp_mtx);
    }
    if (g_comp_stop) { pthread_mutex_unlock(&g_comp_mtx); return NULL; }
    comp_job job = g_comp_job;
    uint64_t my_job_id = g_comp_cur_job_id;
    last_seen_job_id = my_job_id;
    uint32_t y0=w->y0, y1=w->y1;
    pthread_mutex_unlock(&g_comp_mtx);
    composite_layers_rows(job, y0, y1);
    pthread_mutex_lock(&g_comp_mtx);
    if (g_comp_cur_job_id == my_job_id) {
      g_comp_done++;
      if (g_comp_done == g_comp_workers) {
        g_comp_have_job = false;
        pthread_cond_signal(&g_comp_cv_done);
      }
    }
    pthread_mutex_unlock(&g_comp_mtx);
  }
}
static int comp_pool_init(int threads, uint32_t out_h) {
  g_comp_workers = threads;
  g_comp_stop=false;
  g_comp_have_job=false;
  g_comp_done=0;
  g_comp_cur_job_id=0;
  g_comp_ws = (comp_worker*)calloc((size_t)g_comp_workers, sizeof(*g_comp_ws));
  if (!g_comp_ws) return -1;
  int cpu_count = (int)sysconf(_SC_NPROCESSORS_ONLN);
  if (cpu_count < 1) cpu_count = 1;
  for (int i=0;i<g_comp_workers;i++) {
    g_comp_ws[i].id=i;
    g_comp_ws[i].cpu = (i < cpu_count) ? i : (i % cpu_count);
    uint32_t y0=(uint32_t)((uint64_t)out_h*(uint64_t)i/(uint64_t)g_comp_workers);
    uint32_t y1=(uint32_t)((uint64_t)out_h*(uint64_t)(i+1)/(uint64_t)g_comp_workers);
    g_comp_ws[i].y0=y0; g_comp_ws[i].y1=y1;
    if (pthread_create(&g_comp_ws[i].th, NULL, comp_worker_main, &g_comp_ws[i]) != 0) return -1;
  }
  return 0;
}
static void comp_pool_destroy(void) {
  pthread_mutex_lock(&g_comp_mtx);
  g_comp_stop=true;
  pthread_cond_broadcast(&g_comp_cv_job);
  pthread_mutex_unlock(&g_comp_mtx);
  for (int i=0;i<g_comp_workers;i++) pthread_join(g_comp_ws[i].th, NULL);
  free(g_comp_ws);
  g_comp_ws=nullptr;
  g_comp_workers=0;
}
static void comp_pool_run(const comp_job &job_in) {
  pthread_mutex_lock(&g_comp_mtx);
  g_comp_job = job_in;
  g_comp_done=0;
  g_comp_have_job=true;
  g_comp_cur_job_id++;
  g_comp_job.job_id = g_comp_cur_job_id;
  pthread_cond_broadcast(&g_comp_cv_job);
  while (g_comp_have_job) pthread_cond_wait(&g_comp_cv_done, &g_comp_mtx);
  pthread_mutex_unlock(&g_comp_mtx);
}

// ---------------- Global state ----------------
static std::mutex g_cfg_mtx;
static persisted_config g_cfg;
static std::string g_cfg_path="./overlay_config.json";
static std::atomic<bool> g_quit{false};
static std::atomic<bool> g_reinit_requested{false};
static std::mutex g_reinit_mtx;
static std::string g_reinit_reason;
static void request_reinit(const std::string &reason) {
  { std::lock_guard<std::mutex> lk(g_reinit_mtx); g_reinit_reason = reason; }
  g_reinit_requested.store(true);
}
static std::string take_reinit_reason() {
  std::lock_guard<std::mutex> lk(g_reinit_mtx);
  return g_reinit_reason;
}
static persisted_config cfg_snapshot() {
  std::lock_guard<std::mutex> lk(g_cfg_mtx);
  persisted_config c = g_cfg;
  cfg_normalize(c);
  return c;
}
static bool save_config_locked() {
  std::string j = config_to_json(g_cfg);
  return write_file_atomic(g_cfg_path, j);
}

// apply status
static std::mutex g_apply_mtx;
static bool g_last_apply_ok=true;
static std::string g_last_apply_error;
static void set_last_apply(bool ok, const std::string &err) {
  std::lock_guard<std::mutex> lk(g_apply_mtx);
  g_last_apply_ok=ok; g_last_apply_error=err;
}
static void get_last_apply(bool *ok, std::string *err) {
  std::lock_guard<std::mutex> lk(g_apply_mtx);
  if (ok) *ok=g_last_apply_ok;
  if (err) *err=g_last_apply_error;
}

// ---------------- Pipeline ----------------
struct pipeline {
  int drm_fd=-1;
  drmModeConnector *conn=nullptr;
  uint32_t conn_id=0;
  uint32_t crtc_id=0, crtc_index=0, out_w=0, out_h=0;
  uint32_t plane_id=0;
  uint32_t p_fb_id=0,p_crtc_id=0,p_crtc_x=0,p_crtc_y=0,p_crtc_w=0,p_crtc_h=0;
  uint32_t p_src_x=0,p_src_y=0,p_src_w=0,p_src_h=0;
  dumb_fb fb[2];
  int cur=0;
  int comp_threads=0;
  bool saw_frame=false;
  std::vector<std::unique_ptr<source_capture>> sources;
  std::vector<layer_cfg> layers_work_;
  std::vector<layer_map_cache> layer_maps_;
  out_to_vp_lut lut_;
  std::vector<int> layer_source_index_;
  uint64_t last_rendered_seq_any = 0;

  bool init_from_config(persisted_config cfg) {
    cfg_normalize(cfg);
    memset(&fb[0],0,sizeof(fb[0]));
    memset(&fb[1],0,sizeof(fb[1]));
    int cpu_count=(int)sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count<1) cpu_count=1;
    comp_threads = cfg.threads;
    if (comp_threads<=0) comp_threads=cpu_count;
    if (comp_threads<1) comp_threads=1;
    if (comp_threads>cpu_count) comp_threads=cpu_count;

    std::vector<std::string> eff = cfg_effective_sources(cfg);
    fprintf(stderr, "[pipeline] init threads=%d sources=%zu\n", comp_threads, eff.size());

    sources.clear();

    // 1) TC358743 pair in configured order
    for (size_t i=0; i<cfg.tc358743_order.size(); i++) {
      auto cap = std::make_unique<source_capture>();
      if (cap->init_tc358743(cfg, cfg.tc358743_order[i])) {
        sources.push_back(std::move(cap));
      } else {
        fprintf(stderr, "[pipeline] tc358743 source %s not active; skipping init\n", cfg.tc358743_order[i].c_str());
      }
    }

    // 2) Aux sources
    for (auto &a : cfg.aux_sources) {
      if (a.dev.empty()) continue;
      auto cap = std::make_unique<source_capture>();
      if (cap->init_aux(cfg, a)) {
        sources.push_back(std::move(cap));
      } else {
        fprintf(stderr, "[pipeline] aux source %s not active; skipping init\n", a.dev.c_str());
      }
    }

    // 3) Legacy extra sources (only if auxSources empty)
    if (cfg.aux_sources.empty()) {
      for (auto &s : cfg.v4l2_sources) {
        if (s.empty()) continue;
        // already included by tc order?
        bool already=false;
        for (auto &tc : cfg.tc358743_order) if (tc == s) { already=true; break; }
        if (already) continue;
        auto cap = std::make_unique<source_capture>();
        aux_source_cfg aa; aa.dev = s; aa.mode = "auto";
        if (cap->init_aux(cfg, aa)) {
          sources.push_back(std::move(cap));
        }
      }
    }

    if (sources.empty()) {
      fprintf(stderr, "[pipeline] no active sources; init failed\n");
      return false;
    }

    drm_fd = open_vc4_card();
    if (drm_fd < 0) { perror("open_vc4_card"); return false; }
    if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) return false;

    if (find_hdmi_a_1(drm_fd, &conn_id, &conn) != 0) return false;
    if (conn->connection != DRM_MODE_CONNECTED) return false;

    if (get_active_crtc_for_connector(drm_fd, conn, &crtc_id, &crtc_index, &out_w, &out_h) != 0) return false;

    if (cfg.prefer_output_mode) {
      (void)drm_modeset_prefer(drm_fd, conn_id, conn, crtc_id, cfg.prefer_out_w, cfg.prefer_out_h, 60.0);
      (void)get_active_crtc_for_connector(drm_fd, conn, &crtc_id, &crtc_index, &out_w, &out_h);
    }
    if (cfg.do_modeset) {
      uint32_t inw = sources[0]->in_w;
      uint32_t inh = sources[0]->in_h;
      (void)drm_modeset_match_input(drm_fd, conn_id, conn, crtc_id, inw, inh);
      (void)get_active_crtc_for_connector(drm_fd, conn, &crtc_id, &crtc_index, &out_w, &out_h);
    }

    const uint32_t fmt=DRM_FORMAT_XRGB8888;
    plane_id = find_primary_plane_on_crtc(drm_fd, crtc_index, fmt);
    if (!plane_id) return false;

    p_fb_id   = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID");
    p_crtc_id = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID");
    p_crtc_x  = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X");
    p_crtc_y  = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y");
    p_crtc_w  = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W");
    p_crtc_h  = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H");
    p_src_x   = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X");
    p_src_y   = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y");
    p_src_w   = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W");
    p_src_h   = get_prop_id(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H");
    if (!p_fb_id || !p_crtc_id || !p_crtc_x || !p_crtc_y || !p_crtc_w || !p_crtc_h || !p_src_x || !p_src_y || !p_src_w || !p_src_h) return false;

    present_rect scanout{};
    scanout.crtc_x=0; scanout.crtc_y=0;
    scanout.crtc_w=out_w; scanout.crtc_h=out_h;
    scanout.src_w=out_w; scanout.src_h=out_h;

    if (dumb_fb_create(drm_fd, out_w, out_h, fmt, &fb[0]) != 0) return false;
    if (dumb_fb_create(drm_fd, out_w, out_h, fmt, &fb[1]) != 0) return false;

    if (drm_commit_plane_present(drm_fd, plane_id, crtc_id,
                                 p_fb_id,p_crtc_id,p_crtc_x,p_crtc_y,p_crtc_w,p_crtc_h,
                                 p_src_x,p_src_y,p_src_w,p_src_h,
                                 fb[0].fb_id, &scanout) != 0) return false;

    if (comp_pool_init(comp_threads, out_h) != 0) return false;

    cur=0;
    saw_frame=false;
    layers_work_.clear();
    layer_maps_.clear();
    lut_ = {};
    layer_source_index_.clear();
    last_rendered_seq_any = 0;
    return true;
  }

  void shutdown() {
    if (g_comp_workers > 0) comp_pool_destroy();
    if (drm_fd>=0 && plane_id) {
      drmModeAtomicReq *off = drmModeAtomicAlloc();
      if (off) {
        drmModeAtomicAddProperty(off, plane_id, p_fb_id, 0);
        drmModeAtomicAddProperty(off, plane_id, p_crtc_id, 0);
        drmModeAtomicCommit(drm_fd, off, 0, NULL);
        drmModeAtomicFree(off);
      }
    }
    if (drm_fd>=0) {
      dumb_fb_destroy(drm_fd, &fb[0]);
      dumb_fb_destroy(drm_fd, &fb[1]);
    }
    for (auto &s : sources) if (s) s->shutdown();
    sources.clear();
    if (conn) { drmModeFreeConnector(conn); conn=nullptr; }
    if (drm_fd>=0) { close(drm_fd); drm_fd=-1; }
  }

  bool detect_input_change_and_request_reinit() {
    for (auto &s : sources) {
      std::string reason;
      if (s && s->detect_input_change_and_request_reinit(reason)) {
        request_reinit(reason);
        return true;
      }
    }
    return false;
  }

  bool step_capture_publish_blocking(const persisted_config &cfg, int timeout_ms, bool *got_any_new) {
    if (got_any_new) *got_any_new = false;
    bool any=false;
    for (auto &s : sources) {
      if (!s) continue;
      bool got=false;
      if (!s->step_capture_publish_blocking(cfg, timeout_ms, &got)) return false;
      if (got) any=true;
    }
    if (got_any_new) *got_any_new = any;
    return true;
  }

  int resolve_layer_source_index(const persisted_config &cfg_live, const layer_cfg &L, std::string &resolved_dev, bool &active) {
    std::string req = L.video_source.empty() ? cfg_live.v4l2_dev : L.video_source;
    if (req.empty()) req = cfg_live.v4l2_dev;
    resolved_dev = req;
    active = false;
    for (size_t i=0;i<sources.size();i++) {
      if (!sources[i]) continue;
      if (sources[i]->dev == req) {
        source_probe p = sources[i]->probe_snapshot();
        active = p.active;
        if (!active) return -1;
        return (int)i;
      }
    }
    return -1;
  }

  bool step_render_present_if_new(const persisted_config &cfg_live_in) {
    persisted_config cfg_live = cfg_live_in;
    cfg_normalize(cfg_live);

    // Safer than XOR: sum with rotate
    uint64_t any_seq = 0;
    for (auto &s : sources) {
      if (!s) continue;
      uint64_t v = s->latest_seq.load(std::memory_order_acquire);
      any_seq = (any_seq << 1) ^ v ^ (any_seq >> 63);
    }
    if (any_seq == 0 || any_seq == last_rendered_seq_any) return true;

    // Snapshot source frames
    std::vector<comp_job::source_frame> sframes;
    sframes.reserve(sources.size());
    for (auto &s : sources) {
      comp_job::source_frame sf;
      if (s) {
        sf.dev = s->dev;
        latest_frame lf = s->latest_snapshot();
        if (lf.valid && lf.ptr_xrgb) {
          sf.src_xrgb = lf.ptr_xrgb;
          sf.src_stride_bytes = lf.stride_bytes;
          sf.in_w = lf.w;
          sf.in_h = lf.h;
          sf.valid = true;
        }
      }
      sframes.push_back(sf);
    }

    // Choose first valid as "reference" for viewport
    latest_frame lf0{};
    bool have0=false;
    for (auto &s : sources) {
      if (!s) continue;
      latest_frame lf = s->latest_snapshot();
      if (lf.valid && lf.ptr_xrgb) { lf0 = lf; have0=true; break; }
    }
    if (!have0) return true;

    uint32_t vpw=lf0.w, vph=lf0.h;
    if (cfg_live.viewport.set) { vpw=cfg_live.viewport.w; vph=cfg_live.viewport.h; }
    if (!vpw || !vph) { vpw=lf0.w; vph=lf0.h; }

    layers_work_ = cfg_live.layers;
    if (layers_work_.empty()) {
      layer_cfg L;
      L.name="FullFrame";
      L.type=LAYER_VIDEO;
      L.enabled=true;
      L.src_rect={0,0,lf0.w,lf0.h};
      L.dst_pos={0,0};
      L.scale_x=1.0f; L.scale_y=1.0f;
      L.opacity=1.0f;
      L.invert_rel=INV_NONE;
      L.video_source = cfg_live.v4l2_dev;
      layers_work_.push_back(L);
      vpw=lf0.w; vph=lf0.h;
    }

    layer_source_index_.assign(layers_work_.size(), -1);
    std::vector<runtime_info::layer_rt> layer_rt(layers_work_.size());
    for (size_t i=0;i<layers_work_.size();i++) {
      const auto &L = layers_work_[i];
      if (L.type != LAYER_VIDEO) {
        layer_rt[i] = { "", "", true };
        continue;
      }
      std::string resolved;
      bool active=false;
      int idx = resolve_layer_source_index(cfg_live, L, resolved, active);
      layer_source_index_[i] = idx;
      layer_rt[i].requested = L.video_source.empty() ? cfg_live.v4l2_dev : L.video_source;
      layer_rt[i].resolved = resolved;
      layer_rt[i].active = (idx >= 0) && active;
    }

    // Sources runtime info
    std::vector<runtime_info::source_rt> sources_rt;
    auto list_for_status = cfg_effective_sources(cfg_live);
    sources_rt.reserve(list_for_status.size());
    for (const auto &dev : list_for_status) {
      runtime_info::source_rt sr;
      sr.dev = dev;
      bool found=false;
      for (auto &s : sources) {
        if (s && s->dev == dev) {
          source_probe p = s->probe_snapshot();
          sr.present = p.present;
          sr.active = p.active;
          sr.dv_w = p.dv_w; sr.dv_h = p.dv_h; sr.pixelclock = p.pixelclock;
          sr.fmt_w = p.fmt_w; sr.fmt_h = p.fmt_h; sr.stride = p.stride; sr.pixfmt = p.pixfmt;
          sr.err = p.err;
          found=true;
          break;
        }
      }
      if (!found) {
        source_probe p = probe_v4l2_source(dev);
        sr.present = p.present;
        sr.active = p.active;
        sr.dv_w = p.dv_w; sr.dv_h = p.dv_h; sr.pixelclock = p.pixelclock;
        sr.fmt_w = p.fmt_w; sr.fmt_h = p.fmt_h; sr.stride = p.stride; sr.pixfmt = p.pixfmt;
        sr.err = p.err;
      }
      sources_rt.push_back(sr);
    }

    present_rect map_pr = compute_present_rect(cfg_live.present_policy.c_str(), out_w, out_h, vpw, vph);
    {
      std::lock_guard<std::mutex> lk(g_rt.mtx);
      g_rt.out_w = out_w;
      g_rt.out_h = out_h;
      g_rt.vpw = vpw;
      g_rt.vph = vph;
      g_rt.pr = map_pr;
      g_rt.present_policy = cfg_live.present_policy;
      g_rt.sources = sources_rt;
      g_rt.layerSources = layer_rt;
    }

    lut_.rebuild(out_w, out_h, vpw, vph, map_pr);
    layer_maps_.resize(layers_work_.size());
    for (size_t i=0;i<layers_work_.size();i++) {
      int sidx = layer_source_index_[i];
      if (layers_work_[i].type != LAYER_VIDEO) { layer_maps_[i].enabled=false; continue; }
      if (sidx < 0 || (size_t)sidx >= sframes.size() || !sframes[(size_t)sidx].valid) {
        layer_maps_[i].enabled=false;
        continue;
      }
      layer_maps_[i].build_from_layer(layers_work_[i], sframes[(size_t)sidx].in_w, sframes[(size_t)sidx].in_h);
    }

    int next = cur ^ 1;
    comp_job job{};
    job.sources = sframes;
    job.dst_xrgb=(uint8_t*)fb[next].map;
    job.dst_stride=fb[next].pitch;
    job.out_w=out_w; job.out_h=out_h;
    job.map_pr=map_pr;
    job.vpw=vpw; job.vph=vph;
    job.layers=&layers_work_;
    job.lut = &lut_;
    job.layer_maps = &layer_maps_;
    job.layer_source_index = &layer_source_index_;
    comp_pool_run(job);

    if (!saw_frame) { fprintf(stderr, "[pipeline] first composited frame\n"); saw_frame=true; }
    if (drm_plane_flip_fb_only(drm_fd, plane_id, p_fb_id, fb[next].fb_id) != 0) return false;
    cur=next;
    last_rendered_seq_any = any_seq;
    return true;
  }
};

// ---------------- WebUI glue ----------------
static std::atomic<pid_t> g_pid{0};

static std::string status_json() {
  bool ok=false; std::string err;
  get_last_apply(&ok, &err);

  uint32_t outw=0, outh=0, vpw=0, vph=0;
  present_rect pr{};
  std::string policy="fit";
  std::vector<runtime_info::source_rt> sources;
  std::vector<runtime_info::layer_rt> layerSources;
  {
    std::lock_guard<std::mutex> lk(g_rt.mtx);
    outw = g_rt.out_w; outh = g_rt.out_h; vpw = g_rt.vpw; vph = g_rt.vph;
    pr = g_rt.pr; policy = g_rt.present_policy;
    sources = g_rt.sources; layerSources = g_rt.layerSources;
  }

  json j;
  j["pid"] = (int)g_pid.load();
  j["reinitRequested"] = g_reinit_requested.load();
  j["reinitReason"] = take_reinit_reason();
  j["configPath"] = g_cfg_path;
  j["lastApplyOk"] = ok;
  j["lastApplyError"] = err;

  json jsources = json::array();
  for (auto &s : sources) {
    json o;
    o["dev"] = s.dev;
    o["present"] = s.present;
    o["active"] = s.active;
    o["dvW"] = s.dv_w;
    o["dvH"] = s.dv_h;
    o["pixelclock"] = (uint64_t)s.pixelclock;
    o["fmtW"] = s.fmt_w;
    o["fmtH"] = s.fmt_h;
    o["stride"] = s.stride;
    o["pixfmt"] = s.pixfmt;
    if (!s.err.empty()) o["err"] = s.err;
    jsources.push_back(o);
  }

  json jls = json::array();
  for (auto &ls : layerSources) {
    json o;
    o["requested"] = ls.requested;
    o["resolved"] = ls.resolved;
    o["active"] = ls.active;
    jls.push_back(o);
  }

  j["runtime"] = {
    {"outW", outw},
    {"outH", outh},
    {"vpw", vpw},
    {"vph", vph},
    {"presentPolicy", policy},
    {"presentRect", {
      {"crtcX", pr.crtc_x},
      {"crtcY", pr.crtc_y},
      {"crtcW", pr.crtc_w},
      {"crtcH", pr.crtc_h},
      {"srcW",  pr.src_w},
      {"srcH",  pr.src_h},
    }},
    {"sources", jsources},
    {"layerSources", jls},
  };

  return j.dump();
}

static std::string config_json_provider() { return config_to_json(cfg_snapshot()); }

// ---- V4L2 caps provider wiring for /api/v4l2/caps?dev=... ----
static std::string json_error_caps(const std::string &dev, const std::string &msg, int err_no, int http_status) {
  json j;
  j["ok"] = false;
  j["dev"] = dev;
  j["error"] = msg;
  if (err_no != 0) {
    j["errno"] = err_no;
    j["errnoStr"] = std::string(strerror(err_no));
  }
  j["httpStatus"] = http_status;
  return j.dump();
}
static std::string v4l2_caps_provider_by_dev(const std::string &dev) {
  if (dev.empty()) return json_error_caps(dev, "missing 'dev' query parameter", 0, 400);
  if (dev.rfind("/dev/", 0) != 0) return json_error_caps(dev, "dev must be a /dev/videoX path", 0, 400);

  v4l2_device_caps caps = v4l2_query_caps(dev);
  if (!caps.ok) return json_error_caps(dev, caps.err.empty() ? "v4l2_query_caps failed" : caps.err, errno, 500);

  json jf = json::array();
  for (const auto &f : caps.formats) {
    json o;
    o["pixfmt"] = f.pixfmt;
    o["fourcc"] = fourcc_to_string(f.pixfmt);
    o["description"] = f.desc;
    json sizes = json::array();
    for (const auto &sz : f.sizes) sizes.push_back({ {"w", sz.first}, {"h", sz.second} });
    o["sizes"] = sizes;
    jf.push_back(o);
  }

  json j;
  j["ok"] = true;
  j["dev"] = dev;
  j["formats"] = jf;
  return j.dump();
}

// apply: accepts tc358743Order + auxSources + legacy v4l2Sources + existing fields
static bool apply_from_body(const std::string &body, std::string &out_json, int &out_http_status) {
  persisted_config cur = cfg_snapshot();
  persisted_config next = cur;
  json j = json::parse(body, nullptr, false);
  if (j.is_discarded() || !j.is_object()) {
    set_last_apply(false, "invalid json");
    out_http_status = 400;
    out_json = "{\"error\":\"invalid json\"}";
    return false;
  }
  auto get_str = [&](const char *k, std::string &out) {
    if (j.contains(k) && j[k].is_string()) out = j[k].get<std::string>();
  };
  auto get_bool = [&](const char *k, bool &out) {
    if (j.contains(k) && j[k].is_boolean()) out = j[k].get<bool>();
  };
  auto get_int = [&](const char *k, int &out) {
    if (j.contains(k) && j[k].is_number_integer()) out = j[k].get<int>();
  };
  std::string s;
  get_str("v4l2Dev", s); if (!s.empty()) next.v4l2_dev = s;
  get_str("edidPath", s); next.edid_path = s;
  get_bool("modesetMatchInput", next.do_modeset);
  get_int("threads", next.threads);
  get_bool("bgr", next.input_is_bgr);
  get_str("present", s);
  if (!s.empty()) {
    if (!present_policy_valid(s)) { set_last_apply(false,"invalid present"); out_http_status=400; out_json="{\"error\":\"invalid present\"}"; return false; }
    next.present_policy=s;
  }
  get_bool("preferOutputMode", next.prefer_output_mode);
  get_str("preferOutputRes", s);
  if (!s.empty()) {
    uint32_t w=0,h=0;
    if (sscanf(s.c_str(), "%ux%u", &w, &h) != 2 || !w || !h) {
      set_last_apply(false,"invalid preferOutputRes"); out_http_status=400; out_json="{\"error\":\"invalid preferOutputRes\"}"; return false;
    }
    next.prefer_out_w=w; next.prefer_out_h=h;
  }
  get_str("viewport", s);
  if (s.empty()) next.viewport.set=false;
  else {
    uint32_t w=0,h=0;
    if (sscanf(s.c_str(), "%ux%u", &w, &h) != 2 || !w || !h) {
      set_last_apply(false,"invalid viewport"); out_http_status=400; out_json="{\"error\":\"invalid viewport\"}"; return false;
    }
    next.viewport.set=true; next.viewport.w=w; next.viewport.h=h;
  }
  get_str("listenAddr", s);
  if (!s.empty()) next.listen_addr=s;

  // tc358743Order
  if (j.contains("tc358743Order")) {
    if (!j["tc358743Order"].is_array()) { out_http_status=400; out_json="{\"error\":\"tc358743Order must be array\"}"; return false; }
    next.tc358743_order.clear();
    for (const auto &x : j["tc358743Order"]) {
      if (!x.is_string()) { out_http_status=400; out_json="{\"error\":\"tc358743Order entries must be strings\"}"; return false; }
      next.tc358743_order.push_back(x.get<std::string>());
    }
  }

  // auxSources
  if (j.contains("auxSources")) {
    if (!j["auxSources"].is_array()) { out_http_status=400; out_json="{\"error\":\"auxSources must be array\"}"; return false; }
    next.aux_sources.clear();
    for (const auto &x : j["auxSources"]) {
      if (!x.is_object()) continue;
      aux_source_cfg a;
      if (x.contains("dev") && x["dev"].is_string()) a.dev = x["dev"].get<std::string>();
      if (x.contains("mode") && x["mode"].is_string()) a.mode = x["mode"].get<std::string>();
      if (x.contains("width") && x["width"].is_number_integer()) a.width = (uint32_t)x["width"].get<int>();
      if (x.contains("height") && x["height"].is_number_integer()) a.height = (uint32_t)x["height"].get<int>();
      if (x.contains("pixfmt") && x["pixfmt"].is_string()) a.pixfmt = x["pixfmt"].get<std::string>();
      next.aux_sources.push_back(a);
    }
  }

  // legacy v4l2Sources (string array) still accepted
  if (j.contains("v4l2Sources")) {
    if (!j["v4l2Sources"].is_array()) { out_http_status=400; out_json="{\"error\":\"v4l2Sources must be array\"}"; return false; }
    next.v4l2_sources.clear();
    for (const auto &x : j["v4l2Sources"]) {
      if (!x.is_string()) { out_http_status=400; out_json="{\"error\":\"v4l2Sources entries must be strings\"}"; return false; }
      std::string dev = x.get<std::string>();
      if (!dev.empty()) next.v4l2_sources.push_back(dev);
    }
  }

  // Layers: easiest safe path is to roundtrip through overlay_backend JSON parser by building a temp json string.
  if (j.contains("layers") && j["layers"].is_array()) {
    json tmp;
    tmp["layers"] = j["layers"];
    tmp["v4l2Dev"] = next.v4l2_dev;
    tmp["present"] = next.present_policy;
    tmp["bgr"] = next.input_is_bgr;
    tmp["edidPath"] = next.edid_path;
    tmp["threads"] = next.threads;
    tmp["modesetMatchInput"] = next.do_modeset;
    tmp["preferOutputMode"] = next.prefer_output_mode;
    tmp["preferOutputRes"] = std::to_string(next.prefer_out_w) + "x" + std::to_string(next.prefer_out_h);
    tmp["viewport"] = next.viewport.set ? (std::to_string(next.viewport.w) + "x" + std::to_string(next.viewport.h)) : "";
    tmp["listenAddr"] = next.listen_addr;
    tmp["tc358743Order"] = next.tc358743_order;
    tmp["auxSources"] = j.value("auxSources", json::array());
    tmp["v4l2Sources"] = next.v4l2_sources;
    persisted_config reparsed = next;
    if (!config_from_json_text(tmp.dump(), reparsed)) {
      out_http_status=400; out_json="{\"error\":\"invalid layers\"}"; return false;
    }
    next.layers = reparsed.layers;
  }

  cfg_normalize(next);

  bool need_reinit=false; std::string reason;
  if (next.v4l2_dev != cur.v4l2_dev) { need_reinit=true; reason="v4l2Dev changed"; }
  else if (next.tc358743_order != cur.tc358743_order) { need_reinit=true; reason="tc358743Order changed"; }
  else if (next.aux_sources.size() != cur.aux_sources.size()) { need_reinit=true; reason="auxSources changed"; }
  else if (next.edid_path != cur.edid_path) { need_reinit=true; reason="edidPath changed"; }
  else if (next.do_modeset != cur.do_modeset) { need_reinit=true; reason="modesetMatchInput changed"; }
  else if (next.prefer_output_mode != cur.prefer_output_mode) { need_reinit=true; reason="preferOutputMode changed"; }
  else if (next.prefer_out_w != cur.prefer_out_w || next.prefer_out_h != cur.prefer_out_h) { need_reinit=true; reason="preferOutputRes changed"; }
  else if (next.threads != cur.threads) { need_reinit=true; reason="threads changed"; }

  {
    std::lock_guard<std::mutex> lk(g_cfg_mtx);
    g_cfg = next;
    if (!save_config_locked()) {
      set_last_apply(false,"failed to save config file");
      out_http_status=500;
      out_json="{\"error\":\"failed to save config file\"}";
      return false;
    }
  }

  set_last_apply(true,"");
  if (need_reinit) request_reinit(reason);

  json out;
  out["ok"] = true;
  out["saved"] = true;
  out["reinitRequested"] = need_reinit;
  out["reinitReason"] = reason;
  out["effectiveConfig"] = json::parse(config_to_json(next));
  out_json = out.dump();
  out_http_status = 200;
  return true;
}

static bool looks_like_dim(const std::string &s) {
  uint32_t w=0,h=0;
  return (sscanf(s.c_str(), "%ux%u", &w, &h) == 2 && w && h);
}

// ---------------- Reference PNG (minimal uncompressed PNG) ----------------
static uint32_t crc32_table[256];
static void crc32_init() {
  for (uint32_t i=0;i<256;i++){
    uint32_t c=i;
    for(int k=0;k<8;k++) c = (c&1) ? (0xEDB88320u ^ (c>>1)) : (c>>1);
    crc32_table[i]=c;
  }
}
static uint32_t crc32(const uint8_t *buf, size_t len) {
  uint32_t c=0xFFFFFFFFu;
  for (size_t i=0;i<len;i++) c = crc32_table[(c ^ buf[i]) & 0xFF] ^ (c>>8);
  return c ^ 0xFFFFFFFFu;
}
static void png_write_u32(std::vector<uint8_t> &o, uint32_t v) {
  o.push_back((v>>24)&0xFF); o.push_back((v>>16)&0xFF); o.push_back((v>>8)&0xFF); o.push_back(v&0xFF);
}
static void png_write_chunk(std::vector<uint8_t> &o, const char type[4], const std::vector<uint8_t> &data) {
  png_write_u32(o, (uint32_t)data.size());
  size_t start=o.size();
  o.insert(o.end(), type, type+4);
  o.insert(o.end(), data.begin(), data.end());
  uint32_t c = crc32(o.data()+start, 4 + data.size());
  png_write_u32(o, c);
}
static std::vector<uint8_t> xrgb8888_to_png_uncompressed(const uint32_t *xrgb, uint32_t w, uint32_t h, uint32_t stride_bytes) {
  crc32_init();
  std::vector<uint8_t> out;
  const uint8_t sig[8]={137,80,78,71,13,10,26,10};
  out.insert(out.end(), sig, sig+8);

  // IHDR
  std::vector<uint8_t> ihdr;
  png_write_u32(ihdr, w); png_write_u32(ihdr, h);
  ihdr.push_back(8);  // bit depth
  ihdr.push_back(2);  // color type RGB
  ihdr.push_back(0);  // compression
  ihdr.push_back(0);  // filter
  ihdr.push_back(0);  // interlace
  png_write_chunk(out, "IHDR", ihdr);

  // Raw scanlines: filter byte + RGB triplets
  std::vector<uint8_t> raw;
  raw.reserve((size_t)h * (1 + (size_t)w * 3));
  for (uint32_t y=0;y<h;y++) {
    raw.push_back(0); // filter type 0
    const uint8_t *row = (const uint8_t*)xrgb + (uint64_t)y * stride_bytes;
    const uint32_t *px = (const uint32_t*)row;
    for (uint32_t x=0;x<w;x++) {
      uint32_t p = px[x];
      uint8_t r = (uint8_t)((p >> 16) & 0xFF);
      uint8_t g = (uint8_t)((p >> 8) & 0xFF);
      uint8_t b = (uint8_t)(p & 0xFF);
      raw.push_back(r); raw.push_back(g); raw.push_back(b);
    }
  }

  // zlib "stored" blocks (no compression)
  std::vector<uint8_t> z;
  z.push_back(0x78); z.push_back(0x01); // zlib header
  size_t pos=0;
  while (pos < raw.size()) {
    size_t chunk = std::min((size_t)65535, raw.size()-pos);
    bool final = (pos + chunk == raw.size());
    z.push_back(final ? 0x01 : 0x00); // BFINAL + BTYPE=00
    uint16_t len=(uint16_t)chunk;
    uint16_t nlen=(uint16_t)~len;
    z.push_back(len&0xFF); z.push_back((len>>8)&0xFF);
    z.push_back(nlen&0xFF); z.push_back((nlen>>8)&0xFF);
    z.insert(z.end(), raw.begin()+pos, raw.begin()+pos+chunk);
    pos += chunk;
  }

  // Adler32
  uint32_t s1=1,s2=0;
  for (uint8_t b: raw) { s1=(s1+b)%65521; s2=(s2+s1)%65521; }
  uint32_t adler=(s2<<16)|s1;
  png_write_u32(z, adler);

  png_write_chunk(out, "IDAT", z);
  std::vector<uint8_t> empty;
  png_write_chunk(out, "IEND", empty);
  return out;
}

// Grab a single published frame from a source_capture (uses your existing conversion-to-XRGB path)
static bool grab_one_frame_xrgb_from_source(const persisted_config &cfg, source_capture &src,
                                            std::vector<uint32_t> &out_xrgb, uint32_t &out_w, uint32_t &out_h, uint32_t &out_stride_bytes) {
  uint64_t deadline = monotonic_ms() + 1000;
  while (monotonic_ms() < deadline) {
    bool got=false;
    if (!src.step_capture_publish_blocking(cfg, 100, &got)) return false;
    if (!got) continue;
    latest_frame lf = src.latest_snapshot();
    if (!lf.valid || !lf.ptr_xrgb || lf.w == 0 || lf.h == 0 || lf.stride_bytes == 0) continue;

    out_w = lf.w;
    out_h = lf.h;
    out_stride_bytes = lf.w * 4;
    out_xrgb.resize((size_t)out_w * (size_t)out_h);

    // Copy into tight buffer for PNG encoding (ignore lf stride)
    for (uint32_t y=0; y<out_h; y++) {
      const uint8_t *src_row = (const uint8_t*)lf.ptr_xrgb + (uint64_t)y * lf.stride_bytes;
      memcpy((uint8_t*)out_xrgb.data() + (uint64_t)y * out_stride_bytes, src_row, (size_t)out_w * 4);
    }
    return true;
  }
  return false;
}

// ---------------- Main ----------------
static void usage(const char *argv0) {
  fprintf(stderr,
    "Usage: %s [--config PATH] [--no-webui] [--webui-port N] [--listen ADDR]\n"
    "          --v4l2-dev /dev/video0 [--v4l2-src /dev/video1 ...] [--bgr] [--modeset=match-input]\n"
    "          [--present=stretch|fit|1:1] [--edid PATH] [--threads N]\n"
    "          [--prefer-out=WxH|off]\n",
    argv0
  );
}

int main(int argc, char **argv) {
  g_pid.store(getpid());
  enableRawModeNonBlockingStdin();
  bool webui_enabled=true;
  int webui_port=8080;

  for (int i=1;i<argc;i++) {
    if (strcmp(argv[i],"--config")==0 && i+1<argc) g_cfg_path=argv[++i];
  }

  // load config
  {
    persisted_config loaded;
    std::string txt = slurp_file(g_cfg_path);
    if (!txt.empty()) {
      if (config_from_json_text(txt, loaded)) fprintf(stderr, "[config] loaded %s\n", g_cfg_path.c_str());
      else fprintf(stderr, "[config] config parse failed; using defaults\n");
    } else {
      fprintf(stderr, "[config] no config found; using defaults\n");
    }
    cfg_normalize(loaded);
    std::lock_guard<std::mutex> lk(g_cfg_mtx);
    g_cfg = loaded;
  }

  // CLI overrides (legacy)
  for (int i=1;i<argc;i++) {
    if (strcmp(argv[i],"--config")==0) { i++; continue; }
    if (strcmp(argv[i],"--no-webui")==0) { webui_enabled=false; continue; }
    if (strcmp(argv[i],"--webui-port")==0 && i+1<argc) { webui_port=atoi(argv[++i]); continue; }
    if (strcmp(argv[i],"--listen")==0 && i+1<argc) {
      std::lock_guard<std::mutex> lk(g_cfg_mtx);
      g_cfg.listen_addr=argv[++i];
      if (g_cfg.listen_addr.empty() || looks_like_dim(g_cfg.listen_addr)) {
        fprintf(stderr, "[config] invalid listenAddr='%s' -> using 0.0.0.0\n", g_cfg.listen_addr.c_str());
        g_cfg.listen_addr = "0.0.0.0";
      }
      continue;
    }

    std::lock_guard<std::mutex> lk(g_cfg_mtx);
    if (strcmp(argv[i],"--v4l2-dev")==0 && i+1<argc) g_cfg.v4l2_dev=argv[++i];
    else if (strcmp(argv[i],"--v4l2-src")==0 && i+1<argc) g_cfg.v4l2_sources.push_back(argv[++i]);
    else if (strcmp(argv[i],"--modeset=match-input")==0) g_cfg.do_modeset=true;
    else if (strcmp(argv[i],"--bgr")==0) g_cfg.input_is_bgr=true;
    else if (strncmp(argv[i],"--present=",10)==0) {
      std::string p=argv[i]+10;
      if (!present_policy_valid(p)) { fprintf(stderr, "Bad --present\n"); return 1; }
      g_cfg.present_policy=p;
    } else if (strcmp(argv[i],"--edid")==0 && i+1<argc) g_cfg.edid_path=argv[++i];
    else if (strcmp(argv[i],"--threads")==0 && i+1<argc) g_cfg.threads=atoi(argv[++i]);
    else if (strncmp(argv[i],"--prefer-out=",12)==0) {
      std::string v=argv[i]+12;
      if (v=="off") g_cfg.prefer_output_mode=false;
      else {
        uint32_t w=0,h=0;
        if (sscanf(v.c_str(), "%ux%u", &w, &h) != 2 || !w || !h) { fprintf(stderr, "Bad --prefer-out\n"); return 1; }
        g_cfg.prefer_output_mode=true; g_cfg.prefer_out_w=w; g_cfg.prefer_out_h=h;
      }
    } else if (strcmp(argv[i],"-h")==0 || strcmp(argv[i],"--help")==0) {
      usage(argv[0]); return 0;
    }
  }

  {
    std::lock_guard<std::mutex> lk(g_cfg_mtx);
    cfg_normalize(g_cfg);
    (void)save_config_locked();
  }

  pipeline p;
  bool have_pipeline=false;
  for (;;) {
    auto snap=cfg_snapshot();
    if (!present_policy_valid(snap.present_policy)) snap.present_policy="fit";
    if (p.init_from_config(snap)) { have_pipeline=true; break; }
    fprintf(stderr, "[supervisor] pipeline init failed; retrying in 1s...\n");
    p.shutdown();
    usleep(1000*1000);
  }

  // Capture reference PNG once at startup for WebUI preview (/ref.png)
  std::vector<uint8_t> ref_png;
  {
    persisted_config snap = cfg_snapshot();

    // Choose the first active source (pipeline already initialized)
    if (!p.sources.empty() && p.sources[0]) {
      std::vector<uint32_t> xrgb;
      uint32_t w=0,h=0,stride_bytes=0;
      if (grab_one_frame_xrgb_from_source(snap, *p.sources[0], xrgb, w, h, stride_bytes)) {
        ref_png = xrgb8888_to_png_uncompressed(xrgb.data(), w, h, stride_bytes);
        fprintf(stderr, "[webui] reference PNG captured: %zu bytes (%ux%u)\n", ref_png.size(), w, h);
      } else {
        fprintf(stderr, "[webui] reference PNG capture failed\n");
      }
    } else {
      fprintf(stderr, "[webui] reference PNG capture skipped (no sources)\n");
    }
  }

  // WEBUI STARTUP
  if (webui_enabled) {
    fprintf(stderr, "[webui] wiring handlers...\n");
    webui_set_config_json_provider(&config_json_provider);
    webui_set_apply_handler(&apply_from_body);
    webui_set_status_provider(&status_json);
    webui_set_v4l2_caps_provider(&v4l2_caps_provider_by_dev);
    webui_set_quit_flag(&g_quit);
    webui_set_reference_png(ref_png);

    auto snap=cfg_snapshot();
    webui_set_listen_address(snap.listen_addr);
    fprintf(stderr, "[webui] starting server thread on %s:%d ...\n", snap.listen_addr.c_str(), webui_port);

    webui_set_filters_provider([]() -> std::string {
      return filter_registry_json().dump();
    });

    webui_start_detached(webui_port);
    fprintf(stderr, "[webui] open http://<device-ip>:%d/\n", webui_port);
  }

  while (!g_quit.load()) {
    char ch;
    ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n==1 && ch=='q') { g_quit.store(true); break; }

    if (!have_pipeline) {
      auto snap=cfg_snapshot();
      if (!present_policy_valid(snap.present_policy)) snap.present_policy="fit";
      if (!p.init_from_config(snap)) {
        fprintf(stderr, "[supervisor] pipeline init failed; retrying in 1s...\n");
        p.shutdown(); have_pipeline=false;
        usleep(1000*1000);
        continue;
      }
      have_pipeline=true;
      g_reinit_requested.store(false);
      fprintf(stderr, "[supervisor] pipeline running.\n");
    }

    if (g_reinit_requested.load()) {
      std::string reason=take_reinit_reason();
      fprintf(stderr, "[supervisor] reinit requested: %s\n", reason.c_str());
      p.shutdown();
      have_pipeline=false;
      g_reinit_requested.store(false);
      continue;
    }

    auto snap=cfg_snapshot();
    if (p.detect_input_change_and_request_reinit()) continue;

    bool got_new=false;
    if (!p.step_capture_publish_blocking(snap, 1000, &got_new)) {
      fprintf(stderr, "[supervisor] capture failed; reinit...\n");
      p.shutdown(); have_pipeline=false;
      continue;
    }
    if (!got_new) continue;

    if (!p.step_render_present_if_new(snap)) {
      fprintf(stderr, "[supervisor] render failed; reinit...\n");
      p.shutdown(); have_pipeline=false;
      continue;
    }
  }

  fprintf(stderr, "[main] shutting down...\n");
  p.shutdown();
  return 0;
}

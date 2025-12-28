/*
  Optimized compositor version + per-video-layer filter stack (post-scale).

  Added in this version:
  - Per-video-layer filters: layer.filters = [{id, params}, ...]
    * Filters run on the layer’s post-scale grid (layer_map_cache vp_w × vp_h).
    * Implemented filters:
        - mono: grayscale conversion (params: strength 0..1)
        - sobel: edgesOnly (non-edges transparent) or magnitude (grayscale)
            params:
              mode: "edgesOnly" | "magnitude"
              threshold: 0..255  (edgesOnly)
              alpha: 0..1        (edgesOnly, edge opacity)
              invert: bool
  - GET /api/filters: returns filter registry for WebUI dynamic enumeration.

  Notes / constraints:
  - Filters are applied independent of other layers (operate only on the layer’s pixels).
  - Filters currently only apply to VIDEO layers (crosshair/graphics ignored for now).
  - To keep CPU low, we render each filtered video layer into a small scratch buffer
    (layer size), apply its filter chain, then composite into output.
*/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

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

#if defined(__aarch64__) || defined(__ARM_NEON)
  #include <arm_neon.h>
  #define HAVE_NEON 1
#else
  #define HAVE_NEON 0
#endif

#include "third_party/json.hpp"
#include "tc358743_webui.h"

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

// ---------------- parsing helpers ----------------
static bool parse_u32(const char *s, uint32_t *out) {
  if (!s || !*s) return false;
  char *end = NULL;
  errno = 0;
  unsigned long v = strtoul(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') return false;
  if (v > 0xFFFFFFFFul) return false;
  *out = (uint32_t)v;
  return true;
}
static bool parse_dim(const char *s, uint32_t *out_w, uint32_t *out_h) {
  const char *x = (s ? strchr(s, 'x') : NULL);
  if (!s || !*s) return false;
  if (!x) {
    uint32_t d = 0;
    if (!parse_u32(s, &d) || d == 0) return false;
    *out_w = d; *out_h = d;
    return true;
  }
  char a[64], b[64];
  size_t la = (size_t)(x - s);
  size_t lb = strlen(x + 1);
  if (la == 0 || lb == 0 || la >= sizeof(a) || lb >= sizeof(b)) return false;
  memcpy(a, s, la); a[la] = 0;
  memcpy(b, x + 1, lb); b[lb] = 0;
  uint32_t w = 0, h = 0;
  if (!parse_u32(a, &w) || !parse_u32(b, &h)) return false;
  if (w == 0 || h == 0) return false;
  *out_w = w; *out_h = h;
  return true;
}
static bool parse_rgb_csv(const char *s, uint8_t *r, uint8_t *g, uint8_t *b) {
  if (!s) return false;
  int rr=-1, gg=-1, bb=-1;
  if (sscanf(s, "%d,%d,%d", &rr, &gg, &bb) != 3) return false;
  if (rr<0||rr>255||gg<0||gg>255||bb<0||bb>255) return false;
  *r=(uint8_t)rr; *g=(uint8_t)gg; *b=(uint8_t)bb;
  return true;
}
static bool parse_point_csv(const char *s, int32_t *x, int32_t *y) {
  if (!s) return false;
  int xx=0, yy=0;
  if (sscanf(s, "%d,%d", &xx, &yy) != 2) return false;
  *x=(int32_t)xx; *y=(int32_t)yy;
  return true;
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
static int tc358743_set_pixfmt_rgb24(int fd, uint32_t *out_w, uint32_t *out_h, uint32_t *out_stride) {
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
    fprintf(stderr, "[v4l2] VIDIOC_G_FMT failed: %s\n", strerror(errno));
    return -1;
  }
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    fprintf(stderr, "[v4l2] VIDIOC_S_FMT RGB24 failed: %s\n", strerror(errno));
    return -1;
  }
  if (out_w) *out_w = fmt.fmt.pix.width;
  if (out_h) *out_h = fmt.fmt.pix.height;
  if (out_stride) *out_stride = fmt.fmt.pix.bytesperline;
  return 0;
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
static void dump_first_pixels_rgb24(const uint8_t *src) {
  fprintf(stderr, "[v4l2] first pixels @ (0,0..7): ");
  for (int i=0;i<8;i++) {
    const uint8_t *p = src + i*3;
    fprintf(stderr, "%02x%02x%02x ", p[0], p[1], p[2]);
  }
  fprintf(stderr, "\n");
}

// ---------------- DRM scanout helpers ----------------
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
struct runtime_info {
  std::mutex mtx;
  uint32_t out_w = 0;
  uint32_t out_h = 0;
  uint32_t vpw = 0;
  uint32_t vph = 0;
  present_rect pr{};
  std::string present_policy = "fit";
};
static runtime_info g_rt;

static bool present_policy_valid(const std::string &p) {
  return p=="stretch" || p=="fit" || p=="1:1";
}
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

// ---------------- Layers / compositor ----------------
struct rect_u32 { uint32_t x=0,y=0,w=0,h=0; };
struct point_i32 { int32_t x=0,y=0; };

static inline uint32_t invert_xrgb8888(uint32_t xrgb) {
  return (xrgb & 0xFF000000u) | ((xrgb ^ 0x00FFFFFFu) & 0x00FFFFFFu);
}
static inline uint32_t pack_xrgb(uint8_t r, uint8_t g, uint8_t b) {
  return 0xFF000000u | ((uint32_t)r<<16) | ((uint32_t)g<<8) | (uint32_t)b;
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

enum layer_invert_rel { INV_NONE=0, INV_LOWER=1, INV_UPPER=2 };
enum layer_type { LAYER_VIDEO=0, LAYER_CROSSHAIR=1, LAYER_GRAPHICS=2 };

// ---------------- Filter stack config ----------------
enum filter_id { FILTER_MONO=1, FILTER_SOBEL=2, FILTER_UNKNOWN=0 };

struct filter_cfg {
  filter_id id = FILTER_UNKNOWN;
  // common: id string stored separately via json; but we keep parsed here
  // mono params
  float mono_strength = 1.0f; // 0..1
  // sobel params
  enum sobel_mode_t { SOBEL_EDGES_ONLY=0, SOBEL_MAGNITUDE=1 } sobel_mode = SOBEL_EDGES_ONLY;
  uint8_t sobel_threshold = 64; // edgesOnly
  float sobel_alpha = 1.0f;     // edgesOnly
  bool sobel_invert = false;
};

struct crosshair_layer_cfg {
  bool enabled=false;
  uint32_t diam_w=50, diam_h=50;
  bool center_set=false;
  int32_t cx=0, cy=0;
  uint32_t thickness=1;
  bool solid=true;
  uint8_t r=255,g=255,b=255;
  float opacity=1.0f;
  layer_invert_rel invert_rel=INV_NONE;
};

struct layer_cfg {
  std::string name="Layer";
  layer_type type=LAYER_VIDEO;
  bool enabled=true;
  rect_u32 src_rect{0,0,0,0};
  point_i32 dst_pos{0,0};
  float scale_x=1.0f;
  float scale_y=1.0f;
  float opacity=1.0f;
  layer_invert_rel invert_rel=INV_NONE;
  crosshair_layer_cfg xh{};
  std::vector<filter_cfg> filters{};
};

struct viewport_cfg { bool set=false; uint32_t w=0,h=0; };

// ---------------- Crosshair drawing (unchanged) ----------------
struct crosshair_runtime {
  bool enabled=false;
  int32_t out_cx=0,out_cy=0;
  int32_t half_w=0,half_h=0,half_th=0;
};
static crosshair_runtime crosshair_make_runtime(const crosshair_layer_cfg &xh, uint32_t out_w, uint32_t out_h) {
  crosshair_runtime rt{};
  if (!xh.enabled) return rt;
  rt.enabled=true;
  rt.out_cx = xh.center_set ? xh.cx : (int32_t)(out_w/2);
  rt.out_cy = xh.center_set ? xh.cy : (int32_t)(out_h/2);
  rt.half_w = (int32_t)xh.diam_w/2;
  rt.half_h = (int32_t)xh.diam_h/2;
  rt.half_th= (int32_t)(xh.thickness ? xh.thickness : 1)/2;
  return rt;
}
static void composite_crosshair_bounded(uint8_t *dst_xrgb, uint32_t dst_stride, uint32_t out_w, uint32_t out_h,
                                        const crosshair_layer_cfg &cfg, layer_invert_rel inv, uint8_t alpha,
                                        uint32_t y0, uint32_t y1,
                                        const uint32_t *upper_buf) {
  if (!cfg.enabled || alpha==0) return;
  crosshair_runtime rt = crosshair_make_runtime(cfg, out_w, out_h);
  if (!rt.enabled) return;
  auto clampi = [](int32_t v, int32_t lo, int32_t hi){ return v<lo?lo:(v>hi?hi:v); };
  uint32_t solid_px = pack_xrgb(cfg.r, cfg.g, cfg.b);
  int32_t hx0 = rt.out_cx - rt.half_w;
  int32_t hx1 = rt.out_cx + rt.half_w;
  int32_t hy0 = rt.out_cy - rt.half_th;
  int32_t hy1 = rt.out_cy + rt.half_th;
  int32_t vx0 = rt.out_cx - rt.half_th;
  int32_t vx1 = rt.out_cx + rt.half_th;
  int32_t vy0 = rt.out_cy - rt.half_h;
  int32_t vy1 = rt.out_cy + rt.half_h;

  int32_t ys = clampi(hy0, (int32_t)y0, (int32_t)y1-1);
  int32_t ye = clampi(hy1, (int32_t)y0, (int32_t)y1-1);
  int32_t xs = clampi(hx0, 0, (int32_t)out_w-1);
  int32_t xe = clampi(hx1, 0, (int32_t)out_w-1);
  for (int32_t y=ys; y<=ye; y++) {
    uint32_t *drow = (uint32_t*)(dst_xrgb + (uint64_t)y*dst_stride);
    const uint32_t *urow = upper_buf ? (upper_buf + (size_t)y*(size_t)out_w) : nullptr;
    for (int32_t x=xs; x<=xe; x++) {
      uint32_t over = cfg.solid ? solid_px : invert_xrgb8888(drow[x]);
      if (inv == INV_LOWER) over = invert_xrgb8888(drow[x]);
      else if (inv == INV_UPPER) {
        uint32_t base = urow ? urow[x] : 0xFF000000u;
        over = invert_xrgb8888(base);
      }
      drow[x] = blend_over_xrgb(drow[x], over, alpha);
    }
  }

  ys = clampi(vy0, (int32_t)y0, (int32_t)y1-1);
  ye = clampi(vy1, (int32_t)y0, (int32_t)y1-1);
  xs = clampi(vx0, 0, (int32_t)out_w-1);
  xe = clampi(vx1, 0, (int32_t)out_w-1);
  for (int32_t y=ys; y<=ye; y++) {
    uint32_t *drow = (uint32_t*)(dst_xrgb + (uint64_t)y*dst_stride);
    const uint32_t *urow = upper_buf ? (upper_buf + (size_t)y*(size_t)out_w) : nullptr;
    for (int32_t x=xs; x<=xe; x++) {
      uint32_t over = cfg.solid ? solid_px : invert_xrgb8888(drow[x]);
      if (inv == INV_LOWER) over = invert_xrgb8888(drow[x]);
      else if (inv == INV_UPPER) {
        uint32_t base = urow ? urow[x] : 0xFF000000u;
        over = invert_xrgb8888(base);
      }
      drow[x] = blend_over_xrgb(drow[x], over, alpha);
    }
  }
}

// ---------------- Invert upper cache (unchanged) ----------------
struct invert_upper_cache {
  bool enabled=false;
  uint32_t out_w=0,out_h=0;
  std::vector<uint32_t> running_upper;
  std::vector<std::vector<uint32_t>> upper_of_layer;
  void init(uint32_t w, uint32_t h, size_t layer_count) {
    out_w=w; out_h=h;
    running_upper.resize((size_t)w*(size_t)h);
    upper_of_layer.clear();
    upper_of_layer.resize(layer_count);
  }
  void ensure_layer_count(size_t layer_count) {
    if (upper_of_layer.size() != layer_count) {
      upper_of_layer.clear();
      upper_of_layer.resize(layer_count);
    }
  }
  void clear_upper_of_layer_buffers() {
    for (auto &v : upper_of_layer) v.clear();
  }
};

static inline void clear_xrgb_rows_fast(uint8_t *dst_xrgb, uint32_t dst_stride, uint32_t out_w, uint32_t y0, uint32_t y1) {
  for (uint32_t y=y0;y<y1;y++) {
    uint32_t *d = (uint32_t*)(dst_xrgb + (uint64_t)y*dst_stride);
    uint32_t x=0;
#if HAVE_NEON
    uint32x4_t v = vdupq_n_u32(0xFF000000u);
    for (; x + 16 <= out_w; x += 16) {
      vst1q_u32(d + x + 0, v);
      vst1q_u32(d + x + 4, v);
      vst1q_u32(d + x + 8, v);
      vst1q_u32(d + x + 12, v);
    }
    for (; x + 4 <= out_w; x += 4) vst1q_u32(d + x, v);
#endif
    for (; x < out_w; x++) d[x] = 0xFF000000u;
  }
}

struct rect_i32 { int32_t x0=0, y0=0, x1=0, y1=0; }; // [x0,x1) [y0,y1)
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
  const present_rect &pr,
  uint32_t vpw, uint32_t vph,
  int32_t layer_vp_x0, int32_t layer_vp_y0,
  int32_t layer_vp_w,  int32_t layer_vp_h
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

// Output->viewport LUTs
struct out_to_vp_lut {
  uint32_t out_w=0, out_h=0;
  uint32_t vpw=0, vph=0;
  present_rect pr{};
  std::vector<int32_t> ox_to_vx; // out_w
  std::vector<int32_t> oy_to_vy; // out_h
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

static inline uint32_t sample_rgb24_xrgb(const uint8_t *src, uint32_t stride, uint32_t sx, uint32_t sy, bool bgr) {
  const uint8_t *p = src + (uint64_t)sy * stride + (uint64_t)sx * 3ull;
  uint8_t r = bgr ? p[2] : p[0];
  uint8_t g = p[1];
  uint8_t b = bgr ? p[0] : p[2];
  return 0xFF000000u | ((uint32_t)r<<16) | ((uint32_t)g<<8) | (uint32_t)b;
}

#if HAVE_NEON
static inline void write4_opaque_rgb24_to_xrgb(uint32_t *dst, const uint8_t *src, uint32_t stride, uint32_t sy, const uint32_t *sx_arr, bool bgr) {
  uint32_t p0 = sample_rgb24_xrgb(src, stride, sx_arr[0], sy, bgr);
  uint32_t p1 = sample_rgb24_xrgb(src, stride, sx_arr[1], sy, bgr);
  uint32_t p2 = sample_rgb24_xrgb(src, stride, sx_arr[2], sy, bgr);
  uint32_t p3 = sample_rgb24_xrgb(src, stride, sx_arr[3], sy, bgr);
  uint32x4_t v = {p0,p1,p2,p3};
  vst1q_u32(dst, v);
}
#endif

// ---------------- Filters (operate on layer buffers) ----------------
// Buffer format: XRGB8888 packed, row stride = w (not bytes)
struct layer_buf {
  uint32_t w=0, h=0;
  std::vector<uint32_t> px; // size w*h
  void resize(uint32_t W, uint32_t H) {
    w=W; h=H;
    px.resize((size_t)w*(size_t)h);
  }
};

static inline uint8_t luma_u8_from_xrgb(uint32_t p) {
  uint32_t r=(p>>16)&0xFF, g=(p>>8)&0xFF, b=p&0xFF;
  // integer approx: 0.299R + 0.587G + 0.114B
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
    uint32_t mr=y, mg=y, mb=y;
    uint32_t rr = (mr*a + r*(255-a) + 127)/255;
    uint32_t gg = (mg*a + g*(255-a) + 127)/255;
    uint32_t bb = (mb*a + b*(255-a) + 127)/255;
    buf.px[i] = 0xFF000000u | (rr<<16) | (gg<<8) | bb;
  }
}

static void filter_apply_sobel(layer_buf &buf,
                              filter_cfg::sobel_mode_t mode,
                              uint8_t threshold,
                              float alpha_f,
                              bool invert) {
  if (buf.w < 3 || buf.h < 3) {
    // tiny buffer: treat as no-edges; edgesOnly => transparent; magnitude => black
    if (mode == filter_cfg::SOBEL_EDGES_ONLY) {
      std::fill(buf.px.begin(), buf.px.end(), 0x00000000u);
    } else {
      std::fill(buf.px.begin(), buf.px.end(), 0xFF000000u);
    }
    return;
  }

  alpha_f = std::clamp(alpha_f, 0.0f, 1.0f);
  uint8_t edge_a = (uint8_t)std::lround(alpha_f * 255.0f);
  std::vector<uint8_t> lum((size_t)buf.w*(size_t)buf.h);
  for (size_t i=0;i<buf.px.size();i++) lum[i] = luma_u8_from_xrgb(buf.px[i]);

  // output
  std::vector<uint32_t> out(buf.px.size());

  auto idx = [&](int x,int y)->size_t { return (size_t)y*(size_t)buf.w + (size_t)x; };

  // borders
  if (mode == filter_cfg::SOBEL_EDGES_ONLY) {
    uint32_t t = 0x00000000u;
    for (uint32_t x=0;x<buf.w;x++) { out[idx((int)x,0)] = t; out[idx((int)x,(int)buf.h-1)] = t; }
    for (uint32_t y=0;y<buf.h;y++) { out[idx(0,(int)y)] = t; out[idx((int)buf.w-1,(int)y)] = t; }
  } else {
    uint32_t t = 0xFF000000u;
    for (uint32_t x=0;x<buf.w;x++) { out[idx((int)x,0)] = t; out[idx((int)x,(int)buf.h-1)] = t; }
    for (uint32_t y=0;y<buf.h;y++) { out[idx(0,(int)y)] = t; out[idx((int)buf.w-1,(int)y)] = t; }
  }

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

      int mag = abs(gx) + abs(gy); // L1 approx
      if (mag > 255) mag = 255;
      uint8_t m = (uint8_t)mag;
      if (invert) m = (uint8_t)(255 - m);

      if (mode == filter_cfg::SOBEL_EDGES_ONLY) {
        if (m < threshold || edge_a == 0) {
          out[idx((int)x,(int)y)] = 0x00000000u; // fully transparent non-edge
        } else {
          // edge pixel grayscale with alpha=edge_a
          out[idx((int)x,(int)y)] = ((uint32_t)edge_a<<24) | ((uint32_t)m<<16) | ((uint32_t)m<<8) | (uint32_t)m;
        }
      } else {
        // magnitude mode: opaque grayscale
        out[idx((int)x,(int)y)] = 0xFF000000u | ((uint32_t)m<<16) | ((uint32_t)m<<8) | (uint32_t)m;
      }
    }
  }

  buf.px.swap(out);
}

// Apply a filter chain in-place to layer buffer
static void apply_filter_chain(layer_buf &buf, const std::vector<filter_cfg> &filters) {
  for (const auto &f : filters) {
    if (f.id == FILTER_MONO) {
      filter_apply_mono(buf, f.mono_strength);
    } else if (f.id == FILTER_SOBEL) {
      filter_apply_sobel(buf, f.sobel_mode, f.sobel_threshold, f.sobel_alpha, f.sobel_invert);
    }
  }
}

// ---------------- Compositor thread pool ----------------
struct comp_job {
  uint64_t job_id=0;
  const uint8_t *src_rgb=nullptr;
  uint32_t src_stride=0;
  uint32_t in_w=0,in_h=0;
  bool input_is_bgr=false;
  uint8_t *dst_xrgb=nullptr;
  uint32_t dst_stride=0;
  uint32_t out_w=0,out_h=0;
  present_rect map_pr{};
  uint32_t vpw=0,vph=0;
  const std::vector<layer_cfg> *layers=nullptr;
  const invert_upper_cache *upper_cache=nullptr;
  const out_to_vp_lut *lut=nullptr;
  const std::vector<layer_map_cache> *layer_maps=nullptr;
  // scratch buffers per job (shared; partitioned by y bands internally by choosing bands outside)
  // For filtered layers we do per-layer raster to small buffers then composite; handled in worker path
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

// Precompute if a layer has filters
static inline bool layer_has_filters(const layer_cfg &L) {
  return (L.type == LAYER_VIDEO) && !L.filters.empty();
}

// Composite a non-filtered video layer directly (existing optimized path)
// (kept mostly identical to prior behavior)
static void composite_video_direct_rows(const comp_job &job,
                                       const layer_cfg &L,
                                       const layer_map_cache &M,
                                       const uint32_t *upper_buf,
                                       uint32_t y0, uint32_t y1) {
  const out_to_vp_lut &lut = *job.lut;
  const uint8_t a = M.alpha;
  const bool bgr = job.input_is_bgr;

  rect_i32 ob = layer_vp_rect_to_output_bounds(job.map_pr, job.vpw, job.vph, M.dst_x, M.dst_y, M.vp_w, M.vp_h);
  ob = clamp_bounds_to_output_and_band(ob, job.out_w, job.out_h, y0, y1);
  if (rect_empty(ob)) return;

  for (int32_t oy=ob.y0; oy<ob.y1; oy++) {
    int32_t vy = lut.oy_to_vy[(uint32_t)oy];
    if (vy < 0) continue;
    int32_t lvy = vy - M.dst_y;
    if ((uint32_t)lvy >= (uint32_t)M.vp_h) continue;
    uint32_t syi = M.sr.y + (uint32_t)M.y_map[(size_t)lvy];

    uint32_t *drow = (uint32_t*)(job.dst_xrgb + (uint64_t)oy * job.dst_stride);
    const uint32_t *urow = upper_buf ? (upper_buf + (size_t)oy*(size_t)job.out_w) : nullptr;

    const bool opaque_simple = (a == 255 && M.inv == INV_NONE && upper_buf == nullptr);

    int32_t ox = ob.x0;
#if HAVE_NEON
    if (opaque_simple) {
      for (; ox + 4 <= ob.x1; ox += 4) {
        int32_t vx0 = lut.ox_to_vx[(uint32_t)(ox+0)];
        int32_t vx1 = lut.ox_to_vx[(uint32_t)(ox+1)];
        int32_t vx2 = lut.ox_to_vx[(uint32_t)(ox+2)];
        int32_t vx3 = lut.ox_to_vx[(uint32_t)(ox+3)];
        if (vx0 < 0 || vx1 < 0 || vx2 < 0 || vx3 < 0) break;
        int32_t lvx0 = vx0 - M.dst_x;
        int32_t lvx1 = vx1 - M.dst_x;
        int32_t lvx2 = vx2 - M.dst_x;
        int32_t lvx3 = vx3 - M.dst_x;
        if ((uint32_t)lvx0 >= (uint32_t)M.vp_w ||
            (uint32_t)lvx1 >= (uint32_t)M.vp_w ||
            (uint32_t)lvx2 >= (uint32_t)M.vp_w ||
            (uint32_t)lvx3 >= (uint32_t)M.vp_w) break;
        uint32_t sx_arr[4];
        sx_arr[0] = M.sr.x + (uint32_t)M.x_map[(size_t)lvx0];
        sx_arr[1] = M.sr.x + (uint32_t)M.x_map[(size_t)lvx1];
        sx_arr[2] = M.sr.x + (uint32_t)M.x_map[(size_t)lvx2];
        sx_arr[3] = M.sr.x + (uint32_t)M.x_map[(size_t)lvx3];
        write4_opaque_rgb24_to_xrgb(drow + ox, job.src_rgb, job.src_stride, syi, sx_arr, bgr);
      }
    }
#endif
    for (; ox < ob.x1; ox++) {
      int32_t vx = lut.ox_to_vx[(uint32_t)ox];
      if (vx < 0) continue;
      int32_t lvx = vx - M.dst_x;
      if ((uint32_t)lvx >= (uint32_t)M.vp_w) continue;
      uint32_t sxi = M.sr.x + (uint32_t)M.x_map[(size_t)lvx];
      uint32_t over = sample_rgb24_xrgb(job.src_rgb, job.src_stride, sxi, syi, bgr);
      if (M.inv == INV_LOWER) {
        over = invert_xrgb8888(drow[(uint32_t)ox]);
      } else if (M.inv == INV_UPPER) {
        uint32_t base = urow ? urow[(uint32_t)ox] : 0xFF000000u;
        over = invert_xrgb8888(base);
      }
      if (a == 255 && M.inv == INV_NONE) drow[(uint32_t)ox] = over;
      else drow[(uint32_t)ox] = blend_over_xrgb(drow[(uint32_t)ox], over, a);
    }
  }
}

// Filtered-layer path:
// 1) Build layer buffer in layer-local output grid (vp_w×vp_h), XRGB8888 but alpha may be used by Sobel edgesOnly.
// 2) Apply filter chain.
// 3) Composite buffer into output using presentRect mapping (output->vp->layerlocal).
//
// To keep computation bounded: buffer size is vp_w×vp_h only.
static void composite_video_filtered_rows(const comp_job &job,
                                         const layer_cfg &L,
                                         const layer_map_cache &M,
                                         const uint32_t *upper_buf,
                                         uint32_t y0, uint32_t y1) {
  const out_to_vp_lut &lut = *job.lut;
  const bool bgr = job.input_is_bgr;

  rect_i32 ob = layer_vp_rect_to_output_bounds(job.map_pr, job.vpw, job.vph, M.dst_x, M.dst_y, M.vp_w, M.vp_h);
  // We'll compute band-clamped bounds later for final composite; but we need full layer buffer first.
  rect_i32 ob_full = clamp_bounds_to_output_and_band(ob, job.out_w, job.out_h, 0, job.out_h);
  if (rect_empty(ob_full)) return;

  // Build layer-local buffer
  layer_buf buf;
  buf.resize((uint32_t)M.vp_w, (uint32_t)M.vp_h);

  // Rasterize (scale) from source into buf (no opacity/invertRel yet, filters independent)
  for (int32_t lvy=0; lvy<M.vp_h; lvy++) {
    uint32_t syi = M.sr.y + (uint32_t)M.y_map[(size_t)lvy];
    uint32_t *dst = &buf.px[(size_t)lvy*(size_t)buf.w];
#if HAVE_NEON
    // Not doing a complicated SIMD gather here; keep scalar for correctness
#endif
    for (int32_t lvx=0; lvx<M.vp_w; lvx++) {
      uint32_t sxi = M.sr.x + (uint32_t)M.x_map[(size_t)lvx];
      dst[(size_t)lvx] = sample_rgb24_xrgb(job.src_rgb, job.src_stride, sxi, syi, bgr);
    }
  }

  // Apply per-layer filter chain
  apply_filter_chain(buf, L.filters);

  // Now composite into output in this worker's band only (y0..y1)
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
    const uint32_t *urow = upper_buf ? (upper_buf + (size_t)oy*(size_t)job.out_w) : nullptr;

    const uint32_t *srcRow = &buf.px[(size_t)lvy*(size_t)buf.w];

    for (int32_t ox=ob_band.x0; ox<ob_band.x1; ox++) {
      int32_t vx = lut.ox_to_vx[(uint32_t)ox];
      if (vx < 0) continue;
      int32_t lvx = vx - M.dst_x;
      if ((uint32_t)lvx >= (uint32_t)M.vp_w) continue;

      uint32_t over = srcRow[(size_t)lvx];

      // If filter produced alpha (sobel edgesOnly), incorporate it with layer opacity.
      uint8_t over_a = (uint8_t)((over >> 24) & 0xFF);
      if (over_a == 0) continue;
      uint8_t final_a = (uint8_t)((uint32_t)over_a * (uint32_t)layer_a / 255u);

      // Apply invertRel at composite stage (kept consistent with non-filter path)
      if (M.inv == INV_LOWER) {
        over = invert_xrgb8888(drow[(uint32_t)ox]) | (over & 0xFF000000u);
      } else if (M.inv == INV_UPPER) {
        uint32_t base = urow ? urow[(uint32_t)ox] : 0xFF000000u;
        over = invert_xrgb8888(base) | (over & 0xFF000000u);
      }

      // Blend using final_a, but over's RGB from filtered buffer
      uint32_t rgb = over & 0x00FFFFFFu;
      uint32_t over_xrgb = 0xFF000000u | rgb;
      drow[(uint32_t)ox] = blend_over_xrgb(drow[(uint32_t)ox], over_xrgb, final_a);
    }
  }
}

static void composite_layers_rows_optimized(const comp_job &job, uint32_t y0, uint32_t y1) {
  const auto &layers = *job.layers;
  const auto &maps = *job.layer_maps;

  clear_xrgb_rows_fast(job.dst_xrgb, job.dst_stride, job.out_w, y0, y1);

  for (size_t li=0; li<layers.size(); li++) {
    const layer_cfg &L = layers[li];
    if (!L.enabled) continue;

    const uint32_t *upper_buf = nullptr;
    if (job.upper_cache && job.upper_cache->enabled) {
      const auto &u = job.upper_cache->upper_of_layer[li];
      if (!u.empty()) upper_buf = u.data();
    }

    if (L.type == LAYER_CROSSHAIR) {
      uint8_t a = (uint8_t)std::lround(std::clamp(L.xh.opacity, 0.0f, 1.0f) * 255.0f);
      if (a == 0) continue;
      composite_crosshair_bounded(job.dst_xrgb, job.dst_stride, job.out_w, job.out_h,
                                  L.xh, L.xh.invert_rel, a, y0, y1, upper_buf);
      continue;
    }

    if (L.type != LAYER_VIDEO) continue;
    const layer_map_cache &M = maps[li];
    if (!M.enabled) continue;

    if (!L.filters.empty()) {
      composite_video_filtered_rows(job, L, M, upper_buf, y0, y1);
    } else {
      composite_video_direct_rows(job, L, M, upper_buf, y0, y1);
    }
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

    composite_layers_rows_optimized(job, y0, y1);

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

// ---------------- Config + JSON ----------------
struct persisted_config {
  std::string v4l2_dev="/dev/video0";
  std::string edid_path;
  bool do_modeset=false;
  int threads=0;
  bool input_is_bgr=false;
  std::string present_policy="fit";
  bool prefer_output_mode=true;
  uint32_t prefer_out_w=1280;
  uint32_t prefer_out_h=720;
  viewport_cfg viewport{};
  std::vector<layer_cfg> layers{};
  std::string listen_addr="0.0.0.0";
};

// Filter registry (for /api/filters)
static json filter_registry_json() {
  json j;
  json filters = json::array();

  {
    json f;
    f["id"] = "mono";
    f["name"] = "Monochrome";
    f["params"] = { {"strength", 1.0} };
    filters.push_back(f);
  }
  {
    json f;
    f["id"] = "sobel";
    f["name"] = "Sobel";
    f["params"] = { {"mode","edgesOnly"}, {"threshold",64}, {"alpha",1.0}, {"invert",false} };
    filters.push_back(f);
  }

  j["filters"] = filters;
  return j;
}

static std::string config_to_json(const persisted_config &c) {
  json j;
  j["v4l2Dev"] = c.v4l2_dev;
  j["edidPath"] = c.edid_path;
  j["modesetMatchInput"] = c.do_modeset;
  j["threads"] = c.threads;
  j["bgr"] = c.input_is_bgr;
  j["present"] = c.present_policy;
  j["preferOutputMode"] = c.prefer_output_mode;
  j["preferOutputRes"] = std::to_string(c.prefer_out_w) + "x" + std::to_string(c.prefer_out_h);
  j["viewport"] = c.viewport.set ? (std::to_string(c.viewport.w) + "x" + std::to_string(c.viewport.h)) : "";
  j["listenAddr"] = c.listen_addr;

  json layers = json::array();
  for (const auto &L : c.layers) {
    json jl;
    jl["name"] = L.name;
    jl["type"] = (L.type==LAYER_CROSSHAIR) ? "crosshair" : (L.type==LAYER_GRAPHICS ? "graphics" : "video");
    jl["enabled"] = L.enabled;
    jl["opacity"] = std::clamp(L.opacity, 0.0f, 1.0f);
    jl["invertRel"] = (L.invert_rel==INV_UPPER) ? "upper" : (L.invert_rel==INV_LOWER ? "lower" : "none");
    jl["srcRect"] = std::to_string(L.src_rect.x) + "," + std::to_string(L.src_rect.y) + "," + std::to_string(L.src_rect.w) + "," + std::to_string(L.src_rect.h);
    jl["dstPos"] = std::to_string(L.dst_pos.x) + "," + std::to_string(L.dst_pos.y);
    {
      std::ostringstream sc;
      sc << L.scale_x << "," << L.scale_y;
      jl["scale"] = sc.str();
    }

    // filters (video only; always include array for stability)
    json fl = json::array();
    if (L.type == LAYER_VIDEO) {
      for (const auto &F : L.filters) {
        json jf;
        if (F.id == FILTER_MONO) {
          jf["id"] = "mono";
          jf["params"] = { {"strength", std::clamp(F.mono_strength, 0.0f, 1.0f)} };
        } else if (F.id == FILTER_SOBEL) {
          jf["id"] = "sobel";
          jf["params"] = {
            {"mode", (F.sobel_mode == filter_cfg::SOBEL_MAGNITUDE) ? "magnitude" : "edgesOnly"},
            {"threshold", (int)F.sobel_threshold},
            {"alpha", std::clamp(F.sobel_alpha, 0.0f, 1.0f)},
            {"invert", F.sobel_invert}
          };
        } else {
          jf["id"] = "unknown";
          jf["params"] = json::object();
        }
        fl.push_back(jf);
      }
    }
    jl["filters"] = fl;

    // crosshair (kept)
    json xh;
    xh["enabled"] = L.xh.enabled;
    xh["diam"] = std::to_string(L.xh.diam_w) + "x" + std::to_string(L.xh.diam_h);
    xh["center"] = L.xh.center_set ? (std::to_string(L.xh.cx) + "," + std::to_string(L.xh.cy)) : "";
    xh["thickness"] = (int)L.xh.thickness;
    xh["mode"] = L.xh.solid ? "solid" : "invert";
    xh["color"] = std::to_string((int)L.xh.r) + "," + std::to_string((int)L.xh.g) + "," + std::to_string((int)L.xh.b);
    xh["opacity"] = std::clamp(L.xh.opacity, 0.0f, 1.0f);
    xh["invertRel"] = (L.xh.invert_rel==INV_UPPER) ? "upper" : (L.xh.invert_rel==INV_LOWER ? "lower" : "none");
    jl["crosshair"] = xh;

    layers.push_back(jl);
  }
  j["layers"] = layers;
  return j.dump();
}

static bool config_from_json_text(const std::string &text, persisted_config &c) {
  json j = json::parse(text, nullptr, false);
  if (j.is_discarded() || !j.is_object()) return false;
  auto get_str = [&](const char *k, std::string &out) {
    if (j.contains(k) && j[k].is_string()) out = j[k].get<std::string>();
  };
  auto get_bool = [&](const char *k, bool &out) {
    if (j.contains(k) && j[k].is_boolean()) out = j[k].get<bool>();
  };
  auto get_int = [&](const char *k, int &out) {
    if (j.contains(k) && j[k].is_number_integer()) out = j[k].get<int>();
  };
  get_str("v4l2Dev", c.v4l2_dev);
  get_str("edidPath", c.edid_path);
  get_bool("modesetMatchInput", c.do_modeset);
  get_int("threads", c.threads);
  get_bool("bgr", c.input_is_bgr);

  std::string present;
  get_str("present", present);
  if (!present.empty() && present_policy_valid(present)) c.present_policy = present;

  get_bool("preferOutputMode", c.prefer_output_mode);
  std::string preferRes;
  get_str("preferOutputRes", preferRes);
  if (!preferRes.empty()) {
    uint32_t w=0,h=0;
    if (parse_dim(preferRes.c_str(), &w,&h)) { c.prefer_out_w=w; c.prefer_out_h=h; }
  }
  std::string viewport;
  get_str("viewport", viewport);
  if (viewport.empty()) c.viewport.set=false;
  else {
    uint32_t w=0,h=0;
    if (parse_dim(viewport.c_str(), &w,&h)) { c.viewport.set=true; c.viewport.w=w; c.viewport.h=h; }
  }

  get_str("listenAddr", c.listen_addr);

  if (j.contains("layers") && j["layers"].is_array()) {
    c.layers.clear();
    for (const auto &jl : j["layers"]) {
      if (!jl.is_object()) continue;
      layer_cfg L;
      if (jl.contains("name") && jl["name"].is_string()) L.name = jl["name"].get<std::string>();
      std::string type = jl.value("type", "video");
      if (type=="crosshair") L.type = LAYER_CROSSHAIR;
      else if (type=="graphics") L.type = LAYER_GRAPHICS;
      else L.type = LAYER_VIDEO;

      L.enabled = jl.value("enabled", true);
      L.opacity = (float)std::clamp(jl.value("opacity", 1.0), 0.0, 1.0);

      std::string inv = jl.value("invertRel", "none");
      if (inv=="lower") L.invert_rel = INV_LOWER;
      else if (inv=="upper") L.invert_rel = INV_UPPER;
      else L.invert_rel = INV_NONE;

      std::string srcRect = jl.value("srcRect", "0,0,0,0");
      {
        unsigned long x=0,y=0,w=0,h=0;
        if (sscanf(srcRect.c_str(), "%lu,%lu,%lu,%lu", &x,&y,&w,&h)==4 && w>0 && h>0) {
          L.src_rect.x=(uint32_t)x; L.src_rect.y=(uint32_t)y; L.src_rect.w=(uint32_t)w; L.src_rect.h=(uint32_t)h;
        }
      }
      std::string dstPos = jl.value("dstPos", "0,0");
      {
        int x=0,y=0;
        if (sscanf(dstPos.c_str(), "%d,%d", &x,&y)==2) { L.dst_pos.x=x; L.dst_pos.y=y; }
      }
      std::string scale = jl.value("scale", "1.0,1.0");
      {
        double sx=1.0, sy=1.0;
        if (sscanf(scale.c_str(), "%lf,%lf", &sx,&sy)==2 && sx>0.0001 && sy>0.0001) {
          L.scale_x=(float)sx; L.scale_y=(float)sy;
        }
      }

      // filters
      if (jl.contains("filters") && jl["filters"].is_array() && L.type == LAYER_VIDEO) {
        for (const auto &jf : jl["filters"]) {
          if (!jf.is_object()) continue;
          std::string id = jf.value("id", "");
          if (id.empty()) continue;
          filter_cfg fc;
          if (id == "mono") {
            fc.id = FILTER_MONO;
            if (jf.contains("params") && jf["params"].is_object()) {
              double st = jf["params"].value("strength", 1.0);
              fc.mono_strength = (float)std::clamp(st, 0.0, 1.0);
            }
          } else if (id == "sobel") {
            fc.id = FILTER_SOBEL;
            if (jf.contains("params") && jf["params"].is_object()) {
              std::string mode = jf["params"].value("mode", "edgesOnly");
              fc.sobel_mode = (mode == "magnitude") ? filter_cfg::SOBEL_MAGNITUDE : filter_cfg::SOBEL_EDGES_ONLY;
              int th = jf["params"].value("threshold", 64);
              if (th < 0) th = 0;
              if (th > 255) th = 255;
              fc.sobel_threshold = (uint8_t)th;
              double a = jf["params"].value("alpha", 1.0);
              fc.sobel_alpha = (float)std::clamp(a, 0.0, 1.0);
              fc.sobel_invert = jf["params"].value("invert", false);
            }
          } else {
            continue;
          }
          L.filters.push_back(fc);
        }
      }

      // crosshair
      if (jl.contains("crosshair") && jl["crosshair"].is_object()) {
        const auto &xh = jl["crosshair"];
        L.xh.enabled = xh.value("enabled", false);
        std::string diam = xh.value("diam", "50x50");
        { uint32_t w=0,h=0; if (parse_dim(diam.c_str(), &w,&h)) { L.xh.diam_w=w; L.xh.diam_h=h; } }
        std::string center = xh.value("center", "");
        if (center.empty()) L.xh.center_set=false;
        else {
          int32_t cx=0,cy=0;
          if (parse_point_csv(center.c_str(), &cx,&cy)) { L.xh.center_set=true; L.xh.cx=cx; L.xh.cy=cy; }
        }
        int th = xh.value("thickness", 1);
        if (th < 1) th = 1;
        if (th > 99) th = 99;
        L.xh.thickness = (uint32_t)th;
        std::string mode = xh.value("mode", "solid");
        L.xh.solid = (mode != "invert");
        std::string color = xh.value("color", "255,255,255");
        { uint8_t r=255,g=255,b=255; if (parse_rgb_csv(color.c_str(), &r,&g,&b)) { L.xh.r=r; L.xh.g=g; L.xh.b=b; } }
        L.xh.opacity = (float)std::clamp(xh.value("opacity", 1.0), 0.0, 1.0);
        std::string xin = xh.value("invertRel", "none");
        if (xin=="lower") L.xh.invert_rel = INV_LOWER;
        else if (xin=="upper") L.xh.invert_rel = INV_UPPER;
        else L.xh.invert_rel = INV_NONE;
      }

      c.layers.push_back(L);
    }
  }
  return true;
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
  return g_cfg;
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
static std::vector<uint8_t> rgb24_to_png_uncompressed(const uint8_t *rgb, uint32_t w, uint32_t h, uint32_t stride) {
  crc32_init();
  std::vector<uint8_t> out;
  const uint8_t sig[8]={137,80,78,71,13,10,26,10};
  out.insert(out.end(), sig, sig+8);
  std::vector<uint8_t> ihdr;
  png_write_u32(ihdr, w); png_write_u32(ihdr, h);
  ihdr.push_back(8); ihdr.push_back(2); ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
  png_write_chunk(out, "IHDR", ihdr);
  std::vector<uint8_t> raw;
  raw.reserve((size_t)h * (1 + (size_t)w*3));
  for (uint32_t y=0;y<h;y++) {
    raw.push_back(0);
    const uint8_t *row = rgb + (uint64_t)y*stride;
    raw.insert(raw.end(), row, row + (size_t)w*3);
  }
  std::vector<uint8_t> z;
  z.push_back(0x78); z.push_back(0x01);
  size_t pos=0;
  while (pos < raw.size()) {
    size_t chunk = std::min((size_t)65535, raw.size()-pos);
    bool final = (pos + chunk == raw.size());
    z.push_back(final ? 0x01 : 0x00);
    uint16_t len=(uint16_t)chunk;
    uint16_t nlen=(uint16_t)~len;
    z.push_back(len&0xFF); z.push_back((len>>8)&0xFF);
    z.push_back(nlen&0xFF); z.push_back((nlen>>8)&0xFF);
    z.insert(z.end(), raw.begin()+pos, raw.begin()+pos+chunk);
    pos += chunk;
  }
  uint32_t s1=1,s2=0;
  for (uint8_t b: raw) { s1=(s1+b)%65521; s2=(s2+s1)%65521; }
  uint32_t adler=(s2<<16)|s1;
  png_write_u32(z, adler);
  png_write_chunk(out, "IDAT", z);
  std::vector<uint8_t> empty;
  png_write_chunk(out, "IEND", empty);
  return out;
}

// ---------------- Capture publishing ----------------
struct latest_frame {
  const uint8_t *ptr=nullptr;
  uint32_t stride=0;
  uint32_t w=0,h=0;
  uint32_t buf_index=0;
  bool valid=false;
};
static std::mutex g_latest_mtx;
static latest_frame g_latest;
static std::atomic<uint64_t> g_latest_seq{0};

// ---------------- Pipeline ----------------
struct pipeline {
  bool have_held_buf=false;
  struct v4l2_buffer held_buf{};
  int vfd=-1;
  cap_buf *cbufs=nullptr;
  uint32_t cbuf_count=0;
  uint32_t in_w=0,in_h=0,in_stride=0;
  uint32_t dv_w=0,dv_h=0;
  uint64_t last_dv_check_ms=0;
  uint64_t dv_check_interval_ms=500;

  int drm_fd=-1;
  drmModeConnector *conn=nullptr;
  uint32_t conn_id=0;
  uint32_t crtc_id=0, crtc_index=0, out_w=0, out_h=0;
  uint32_t plane_id=0;
  uint32_t p_fb_id=0,p_crtc_id=0,p_crtc_x=0,p_crtc_y=0,p_crtc_w=0,p_crtc_h=0;
  uint32_t p_src_x=0,p_src_y=0,p_src_w=0,p_src_h=0;

  dumb_fb fb[2];
  int cur=0;
  invert_upper_cache upper_cache{};
  int comp_threads=0;
  bool dumped=false;
  bool saw_frame=false;

  std::vector<layer_cfg> layers_work_;
  std::vector<layer_map_cache> layer_maps_;
  out_to_vp_lut lut_;
  uint64_t last_rendered_seq = 0;

  bool init_from_config(const persisted_config &cfg) {
    memset(&fb[0],0,sizeof(fb[0]));
    memset(&fb[1],0,sizeof(fb[1]));

    int cpu_count=(int)sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count<1) cpu_count=1;
    comp_threads = cfg.threads;
    if (comp_threads<=0) comp_threads=cpu_count;
    if (comp_threads<1) comp_threads=1;
    if (comp_threads>cpu_count) comp_threads=cpu_count;

    fprintf(stderr, "[pipeline] init v4l2=%s threads=%d\n", cfg.v4l2_dev.c_str(), comp_threads);

    vfd = v4l2_open(cfg.v4l2_dev.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (vfd < 0) { perror("v4l2_open"); return false; }

    if (!cfg.edid_path.empty()) {
      (void)v4l2_set_edid_if_requested(vfd, cfg.edid_path.c_str());
      usleep(1500*1000);
    }

    uint64_t pixelclock=0;
    dv_w=dv_h=0;
    if (tc358743_query_and_set_dv_timings(vfd, &dv_w, &dv_h, &pixelclock, true) != 0) return false;
    if (tc358743_set_pixfmt_rgb24(vfd, &in_w, &in_h, &in_stride) != 0) return false;
    if (v4l2_start_mmap_capture(vfd, &cbufs, &cbuf_count) != 0) return false;

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
      (void)drm_modeset_match_input(drm_fd, conn_id, conn, crtc_id, in_w, in_h);
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

    upper_cache.init(out_w, out_h, 0);
    cur=0;
    dumped=false;
    saw_frame=false;
    have_held_buf=false;
    memset(&held_buf, 0, sizeof(held_buf));
    layers_work_.clear();
    layer_maps_.clear();
    lut_ = {};
    {
      std::lock_guard<std::mutex> lk(g_latest_mtx);
      g_latest = {};
      g_latest.valid=false;
    }
    g_latest_seq.store(0);
    last_rendered_seq = 0;
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

    have_held_buf=false;
    memset(&held_buf, 0, sizeof(held_buf));

    if (vfd>=0) {
      v4l2_stop_and_unmap(vfd, cbufs, cbuf_count);
      cbufs=nullptr; cbuf_count=0;
      v4l2_close(vfd);
      vfd=-1;
    }
    if (conn) { drmModeFreeConnector(conn); conn=nullptr; }
    if (drm_fd>=0) { close(drm_fd); drm_fd=-1; }
  }

  bool detect_input_change_and_request_reinit() {
    uint32_t cur_w=0,cur_h=0,cur_stride=0,cur_pixfmt=0;
    if (v4l2_get_current_fmt(vfd, &cur_w,&cur_h,&cur_stride,&cur_pixfmt)==0) {
      if (cur_w && cur_h && (cur_w!=in_w || cur_h!=in_h || cur_stride!=in_stride)) {
        request_reinit("input resolution changed (VIDIOC_G_FMT)");
        return true;
      }
    }
    uint64_t now=monotonic_ms();
    if ((now-last_dv_check_ms) < dv_check_interval_ms) return false;
    last_dv_check_ms = now;
    uint32_t tw=0,th=0; uint64_t pc=0;
    if (tc358743_query_dv_timings(vfd, &tw,&th,&pc)==0) {
      if (tw && th && (tw!=dv_w || th!=dv_h)) {
        request_reinit("input resolution changed (DV timings)");
        return true;
      }
    }
    return false;
  }

  bool step_capture_publish_blocking(int timeout_ms, bool *got_new_frame) {
    if (got_new_frame) *got_new_frame = false;
    uint64_t before_seq = g_latest_seq.load(std::memory_order_acquire);

    int prc = v4l2_wait_for_frame(vfd, timeout_ms);
    if (prc < 0) return false;
    if (prc == 0) return true;

    struct v4l2_buffer b;
    int dq=v4l2_dequeue_latest_frame(vfd, &b);
    if (dq==1) return true;
    if (dq<0) return false;

    const uint8_t *src = (const uint8_t*)cbufs[b.index].start;
    if (!dumped) { dump_first_pixels_rgb24(src); dumped=true; }

    if (have_held_buf) (void)v4l2_queue_frame(vfd, &held_buf);
    held_buf = b;
    have_held_buf = true;

    {
      std::lock_guard<std::mutex> lk(g_latest_mtx);
      g_latest.ptr=src;
      g_latest.stride=in_stride;
      g_latest.w=in_w; g_latest.h=in_h;
      g_latest.buf_index=b.index;
      g_latest.valid=true;
    }
    g_latest_seq.fetch_add(1, std::memory_order_release);
    uint64_t after_seq = g_latest_seq.load(std::memory_order_acquire);
    if (got_new_frame && after_seq != before_seq) *got_new_frame = true;
    return true;
  }

  void build_upper_cache_fast(const latest_frame &lf, const present_rect &map_pr, uint32_t vpw, uint32_t vph, const persisted_config &cfg_live) {
    // Upper cache currently does NOT incorporate filtered outputs.
    // This keeps it simple and cheap; INV_UPPER with filtered layers will invert against
    // the "unfiltered upper composite". If you need perfect semantics later, we can extend it.
    upper_cache.enabled = true;
    std::fill(upper_cache.running_upper.begin(), upper_cache.running_upper.end(), 0xFF000000u);
    upper_cache.clear_upper_of_layer_buffers();

    out_to_vp_lut ulut;
    ulut.rebuild(out_w, out_h, vpw, vph, map_pr);

    std::vector<layer_map_cache> umaps(layers_work_.size());
    for (size_t i=0;i<layers_work_.size();i++) umaps[i].build_from_layer(layers_work_[i], lf.w, lf.h);

    for (int li=(int)layers_work_.size()-1; li>=0; li--) {
      const layer_cfg &L = layers_work_[(size_t)li];
      bool wants=false;
      if (L.enabled && L.type==LAYER_VIDEO && L.invert_rel==INV_UPPER) wants=true;
      if (L.enabled && L.type==LAYER_CROSSHAIR && L.xh.enabled && L.xh.invert_rel==INV_UPPER) wants=true;
      if (wants) upper_cache.upper_of_layer[(size_t)li] = upper_cache.running_upper;

      if (!L.enabled) continue;

      if (L.type == LAYER_CROSSHAIR) {
        uint8_t a = (uint8_t)std::lround(std::clamp(L.xh.opacity, 0.0f, 1.0f) * 255.0f);
        if (a==0) continue;
        composite_crosshair_bounded((uint8_t*)upper_cache.running_upper.data(), out_w*4, out_w, out_h,
                                   L.xh, INV_NONE, a, 0, out_h, nullptr);
        continue;
      }

      if (L.type != LAYER_VIDEO) continue;
      const layer_map_cache &M = umaps[(size_t)li];
      if (!M.enabled) continue;
      uint8_t a = M.alpha;
      if (a == 0) continue;

      rect_i32 ob = layer_vp_rect_to_output_bounds(map_pr, vpw, vph, M.dst_x, M.dst_y, M.vp_w, M.vp_h);
      ob = clamp_bounds_to_output_and_band(ob, out_w, out_h, 0, out_h);
      if (rect_empty(ob)) continue;

      for (int32_t oy=ob.y0; oy<ob.y1; oy++) {
        int32_t vy = ulut.oy_to_vy[(uint32_t)oy];
        if (vy < 0) continue;
        int32_t lvy = vy - M.dst_y;
        if ((uint32_t)lvy >= (uint32_t)M.vp_h) continue;
        uint32_t syi = M.sr.y + (uint32_t)M.y_map[(size_t)lvy];
        uint32_t *drow = &upper_cache.running_upper[(size_t)oy*(size_t)out_w];
        for (int32_t ox=ob.x0; ox<ob.x1; ox++) {
          int32_t vx = ulut.ox_to_vx[(uint32_t)ox];
          if (vx < 0) continue;
          int32_t lvx = vx - M.dst_x;
          if ((uint32_t)lvx >= (uint32_t)M.vp_w) continue;
          uint32_t sxi = M.sr.x + (uint32_t)M.x_map[(size_t)lvx];
          uint32_t over = sample_rgb24_xrgb(lf.ptr, lf.stride, sxi, syi, cfg_live.input_is_bgr);
          drow[(uint32_t)ox] = blend_over_xrgb(drow[(uint32_t)ox], over, a);
        }
      }
    }
  }

  bool step_render_present_if_new(const persisted_config &cfg_live) {
    uint64_t seq = g_latest_seq.load(std::memory_order_acquire);
    if (seq == 0 || seq == last_rendered_seq) return true;

    latest_frame lf;
    {
      std::lock_guard<std::mutex> lk(g_latest_mtx);
      lf = g_latest;
    }
    if (!lf.valid || !lf.ptr) return true;

    uint32_t vpw=lf.w, vph=lf.h;
    if (cfg_live.viewport.set) { vpw=cfg_live.viewport.w; vph=cfg_live.viewport.h; }
    if (!vpw || !vph) { vpw=lf.w; vph=lf.h; }

    layers_work_ = cfg_live.layers;
    if (layers_work_.empty()) {
      layer_cfg L;
      L.name="FullFrame";
      L.type=LAYER_VIDEO;
      L.enabled=true;
      L.src_rect={0,0,lf.w,lf.h};
      L.dst_pos={0,0};
      L.scale_x=1.0f; L.scale_y=1.0f;
      L.opacity=1.0f;
      L.invert_rel=INV_NONE;
      layers_work_.push_back(L);
      vpw=lf.w; vph=lf.h;
    }
    for (auto &L : layers_work_) {
      if (L.type == LAYER_CROSSHAIR) L.xh.enabled = L.enabled;
    }

    upper_cache.ensure_layer_count(layers_work_.size());

    bool need_upper=false;
    for (auto &L : layers_work_) {
      if (!L.enabled) continue;
      if (L.invert_rel == INV_UPPER) need_upper=true;
      if (L.type==LAYER_CROSSHAIR && L.xh.enabled && L.xh.invert_rel==INV_UPPER) need_upper=true;
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
    }

    lut_.rebuild(out_w, out_h, vpw, vph, map_pr);

    layer_maps_.resize(layers_work_.size());
    for (size_t i=0;i<layers_work_.size();i++) {
      if (layers_work_[i].type == LAYER_CROSSHAIR) { layer_maps_[i].enabled=false; continue; }
      layer_maps_[i].build_from_layer(layers_work_[i], lf.w, lf.h);
    }

    if (need_upper) {
      build_upper_cache_fast(lf, map_pr, vpw, vph, cfg_live);
    } else {
      upper_cache.enabled=false;
      upper_cache.clear_upper_of_layer_buffers();
    }

    int next = cur ^ 1;
    comp_job job{};
    job.src_rgb=lf.ptr;
    job.src_stride=lf.stride;
    job.in_w=lf.w; job.in_h=lf.h;
    job.input_is_bgr=cfg_live.input_is_bgr;
    job.dst_xrgb=(uint8_t*)fb[next].map;
    job.dst_stride=fb[next].pitch;
    job.out_w=out_w; job.out_h=out_h;
    job.map_pr=map_pr;
    job.vpw=vpw; job.vph=vph;
    job.layers=&layers_work_;
    job.upper_cache = (need_upper ? &upper_cache : nullptr);
    job.lut = &lut_;
    job.layer_maps = &layer_maps_;
    comp_pool_run(job);

    if (!saw_frame) { fprintf(stderr, "[pipeline] first composited frame\n"); saw_frame=true; }
    if (drm_plane_flip_fb_only(drm_fd, plane_id, p_fb_id, fb[next].fb_id) != 0) return false;

    cur=next;
    last_rendered_seq = seq;
    return true;
  }
};

// ---------------- Reference frame grabber ----------------
static bool grab_one_frame_rgb24(int vfd, cap_buf *cbufs, uint32_t cbuf_count,
                                 uint32_t in_stride, uint32_t in_w, uint32_t in_h,
                                 std::vector<uint8_t> &out_rgb24) {
  (void)cbuf_count;
  uint64_t deadline = monotonic_ms() + 1000;
  while (monotonic_ms() < deadline) {
    int prc = v4l2_wait_for_frame(vfd, 100);
    if (prc <= 0) continue;
    struct v4l2_buffer b;
    int dq = v4l2_dequeue_latest_frame(vfd, &b);
    if (dq != 0) continue;
    const uint8_t *src = (const uint8_t*)cbufs[b.index].start;
    out_rgb24.resize((size_t)in_w*(size_t)in_h*3);
    for (uint32_t y=0;y<in_h;y++) {
      memcpy(out_rgb24.data() + (size_t)y*(size_t)in_w*3, src + (uint64_t)y*in_stride, (size_t)in_w*3);
    }
    (void)v4l2_queue_frame(vfd, &b);
    return true;
  }
  return false;
}

// ---------------- WebUI glue ----------------
static std::atomic<pid_t> g_pid{0};

static std::string status_json() {
  bool ok=false; std::string err;
  get_last_apply(&ok, &err);
  uint32_t outw=0, outh=0, vpw=0, vph=0;
  present_rect pr{};
  std::string policy="fit";
  {
    std::lock_guard<std::mutex> lk(g_rt.mtx);
    outw = g_rt.out_w;
    outh = g_rt.out_h;
    vpw  = g_rt.vpw;
    vph  = g_rt.vph;
    pr   = g_rt.pr;
    policy = g_rt.present_policy;
  }
  json j;
  j["pid"] = (int)g_pid.load();
  j["reinitRequested"] = g_reinit_requested.load();
  j["reinitReason"] = take_reinit_reason();
  j["configPath"] = g_cfg_path;
  j["lastApplyOk"] = ok;
  j["lastApplyError"] = err;
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
    }}
  };
  return j.dump();
}

// apply_from_body updated: accepts/validates filters
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
    if (!parse_dim(s.c_str(), &w,&h)) { set_last_apply(false,"invalid preferOutputRes"); out_http_status=400; out_json="{\"error\":\"invalid preferOutputRes\"}"; return false; }
    next.prefer_out_w=w; next.prefer_out_h=h;
  }
  get_str("viewport", s);
  if (s.empty()) next.viewport.set=false;
  else {
    uint32_t w=0,h=0;
    if (!parse_dim(s.c_str(), &w,&h)) { set_last_apply(false,"invalid viewport"); out_http_status=400; out_json="{\"error\":\"invalid viewport\"}"; return false; }
    next.viewport.set=true; next.viewport.w=w; next.viewport.h=h;
  }
  get_str("listenAddr", s);
  if (!s.empty()) next.listen_addr=s;

  if (j.contains("layers") && j["layers"].is_array()) {
    next.layers.clear();
    for (const auto &jl : j["layers"]) {
      if (!jl.is_object()) continue;
      layer_cfg L;

      if (jl.contains("name") && jl["name"].is_string()) L.name = jl["name"].get<std::string>();

      std::string type = jl.value("type", "video");
      if (type=="crosshair") L.type = LAYER_CROSSHAIR;
      else if (type=="graphics") L.type = LAYER_GRAPHICS;
      else L.type = LAYER_VIDEO;

      L.enabled = jl.value("enabled", true);

      double op = jl.value("opacity", 1.0);
      if (!(op >= 0.0 && op <= 1.0)) { out_http_status=400; out_json="{\"error\":\"invalid layer opacity\"}"; return false; }
      L.opacity = (float)op;

      std::string inv = jl.value("invertRel", "none");
      if (!(inv=="none"||inv=="lower"||inv=="upper")) { out_http_status=400; out_json="{\"error\":\"invalid layer invertRel\"}"; return false; }
      if (inv=="lower") L.invert_rel = INV_LOWER;
      else if (inv=="upper") L.invert_rel = INV_UPPER;
      else L.invert_rel = INV_NONE;

      std::string srcRect = jl.value("srcRect", "");
      if (srcRect.empty()) { out_http_status=400; out_json="{\"error\":\"invalid layer srcRect\"}"; return false; }
      {
        unsigned long x=0,y=0,w=0,h=0;
        if (sscanf(srcRect.c_str(), "%lu,%lu,%lu,%lu", &x,&y,&w,&h)!=4 || w==0 || h==0) {
          out_http_status=400; out_json="{\"error\":\"invalid layer srcRect\"}"; return false;
        }
        L.src_rect.x=(uint32_t)x; L.src_rect.y=(uint32_t)y; L.src_rect.w=(uint32_t)w; L.src_rect.h=(uint32_t)h;
      }

      std::string dstPos = jl.value("dstPos", "");
      if (dstPos.empty()) { out_http_status=400; out_json="{\"error\":\"invalid layer dstPos\"}"; return false; }
      {
        int x=0,y=0;
        if (sscanf(dstPos.c_str(), "%d,%d", &x,&y)!=2) { out_http_status=400; out_json="{\"error\":\"invalid layer dstPos\"}"; return false; }
        L.dst_pos.x=x; L.dst_pos.y=y;
      }

      std::string scale = jl.value("scale", "");
      if (scale.empty()) { out_http_status=400; out_json="{\"error\":\"invalid layer scale\"}"; return false; }
      {
        double sx=1.0, sy=1.0;
        if (sscanf(scale.c_str(), "%lf,%lf", &sx,&sy)!=2 || !(sx>0.0001 && sy>0.0001)) { out_http_status=400; out_json="{\"error\":\"invalid layer scale\"}"; return false; }
        L.scale_x=(float)sx; L.scale_y=(float)sy;
      }

      // filters (video only)
      if (L.type == LAYER_VIDEO && jl.contains("filters")) {
        if (!jl["filters"].is_array()) { out_http_status=400; out_json="{\"error\":\"filters must be array\"}"; return false; }
        for (const auto &jf : jl["filters"]) {
          if (!jf.is_object()) { out_http_status=400; out_json="{\"error\":\"invalid filter entry\"}"; return false; }
          std::string fid = jf.value("id", "");
          if (fid.empty()) { out_http_status=400; out_json="{\"error\":\"filter id required\"}"; return false; }
          if (!jf.contains("params") || !jf["params"].is_object()) {
            // allow missing params -> defaults
          }
          json params = (jf.contains("params") && jf["params"].is_object()) ? jf["params"] : json::object();

          if (fid == "mono") {
            filter_cfg fc;
            fc.id = FILTER_MONO;
            double st = params.value("strength", 1.0);
            if (!(st>=0.0 && st<=1.0)) { out_http_status=400; out_json="{\"error\":\"mono.strength 0..1\"}"; return false; }
            fc.mono_strength = (float)st;
            L.filters.push_back(fc);
          } else if (fid == "sobel") {
            filter_cfg fc;
            fc.id = FILTER_SOBEL;

            std::string mode = params.value("mode", "edgesOnly");
            if (!(mode=="edgesOnly" || mode=="magnitude")) { out_http_status=400; out_json="{\"error\":\"sobel.mode invalid\"}"; return false; }
            fc.sobel_mode = (mode=="magnitude") ? filter_cfg::SOBEL_MAGNITUDE : filter_cfg::SOBEL_EDGES_ONLY;

            int th = params.value("threshold", 64);
            if (th < 0 || th > 255) { out_http_status=400; out_json="{\"error\":\"sobel.threshold 0..255\"}"; return false; }
            fc.sobel_threshold = (uint8_t)th;

            double a = params.value("alpha", 1.0);
            if (!(a>=0.0 && a<=1.0)) { out_http_status=400; out_json="{\"error\":\"sobel.alpha 0..1\"}"; return false; }
            fc.sobel_alpha = (float)a;

            // IMPORTANT FIX:
            // Do NOT touch params["invert"] before checking contains("invert"),
            // because operator[] will create a null entry and make contains() true.
            if (params.contains("invert")) {
              if (!params["invert"].is_boolean()) { out_http_status=400; out_json="{\"error\":\"sobel.invert bool\"}"; return false; }
              fc.sobel_invert = params["invert"].get<bool>();
            } else {
              fc.sobel_invert = false;
            }

            L.filters.push_back(fc);
          } else {
            out_http_status=400;
            out_json="{\"error\":\"unknown filter id\"}";
            return false;
          }
        }
      }

      // crosshair (unchanged validation)
      if (jl.contains("crosshair") && jl["crosshair"].is_object()) {
        const auto &xh = jl["crosshair"];
        L.xh.enabled = xh.value("enabled", false);

        std::string diam = xh.value("diam", "50x50");
        { uint32_t w=0,h=0; if (!parse_dim(diam.c_str(), &w,&h)) { out_http_status=400; out_json="{\"error\":\"invalid crosshair diam\"}"; return false; } L.xh.diam_w=w; L.xh.diam_h=h; }

        std::string center = xh.value("center", "");
        if (center.empty()) L.xh.center_set=false;
        else {
          int32_t cx=0,cy=0;
          if (!parse_point_csv(center.c_str(), &cx,&cy)) { out_http_status=400; out_json="{\"error\":\"invalid crosshair center\"}"; return false; }
          L.xh.center_set=true; L.xh.cx=cx; L.xh.cy=cy;
        }

        int th = xh.value("thickness", 1);
        if (th<1 || th>99) { out_http_status=400; out_json="{\"error\":\"invalid crosshair thickness\"}"; return false; }
        L.xh.thickness=(uint32_t)th;

        std::string mode = xh.value("mode", "solid");
        if (!(mode=="solid"||mode=="invert")) { out_http_status=400; out_json="{\"error\":\"invalid crosshair mode\"}"; return false; }
        L.xh.solid = (mode=="solid");

        std::string color = xh.value("color", "255,255,255");
        { uint8_t rr=0,gg=0,bb=0; if(!parse_rgb_csv(color.c_str(),&rr,&gg,&bb)){ out_http_status=400; out_json="{\"error\":\"invalid crosshair color\"}"; return false; } L.xh.r=rr; L.xh.g=gg; L.xh.b=bb; }

        double xop = xh.value("opacity", 1.0);
        if (!(xop>=0.0 && xop<=1.0)) { out_http_status=400; out_json="{\"error\":\"invalid crosshair opacity\"}"; return false; }
        L.xh.opacity=(float)xop;

        std::string xin = xh.value("invertRel", "none");
        if (!(xin=="none"||xin=="lower"||xin=="upper")) { out_http_status=400; out_json="{\"error\":\"invalid crosshair invertRel\"}"; return false; }
        if (xin=="lower") L.xh.invert_rel = INV_LOWER;
        else if (xin=="upper") L.xh.invert_rel = INV_UPPER;
        else L.xh.invert_rel = INV_NONE;
      }

      next.layers.push_back(L);
    }
  }

  bool need_reinit=false; std::string reason;
  if (next.v4l2_dev != cur.v4l2_dev) { need_reinit=true; reason="v4l2Dev changed"; }
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

static std::string config_json_provider() {
  return config_to_json(cfg_snapshot());
}

static bool looks_like_dim(const std::string &s) {
  uint32_t w=0,h=0;
  return parse_dim(s.c_str(), &w, &h);
}

// ---------------- Main ----------------
static void usage(const char *argv0) {
  fprintf(stderr,
    "Usage: %s [--config PATH] [--no-webui] [--webui-port N] [--listen ADDR]\n"
    "          --v4l2-dev /dev/video0 [--bgr] [--modeset=match-input]\n"
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
      if (config_from_json_text(txt, loaded)) {
        fprintf(stderr, "[config] loaded %s\n", g_cfg_path.c_str());
      } else {
        fprintf(stderr, "[config] config parse failed; using defaults\n");
      }
    } else {
      fprintf(stderr, "[config] no config found; using defaults\n");
    }
    std::lock_guard<std::mutex> lk(g_cfg_mtx);
    g_cfg = loaded;
  }

  // CLI overrides
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
        if (!parse_dim(v.c_str(), &w,&h)) { fprintf(stderr, "Bad --prefer-out\n"); return 1; }
        g_cfg.prefer_output_mode=true; g_cfg.prefer_out_w=w; g_cfg.prefer_out_h=h;
      }
    } else if (strcmp(argv[i],"-h")==0 || strcmp(argv[i],"--help")==0) {
      usage(argv[0]); return 0;
    } else {
      fprintf(stderr, "Unknown arg: %s\n", argv[i]);
      usage(argv[0]); return 1;
    }
  }

  // persist current config
  {
    std::lock_guard<std::mutex> lk(g_cfg_mtx);
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

  // capture reference png
  std::vector<uint8_t> ref_png;
  {
    uint64_t start=monotonic_ms();
    while (monotonic_ms()-start < 5000 && !g_quit.load()) {
      if (p.detect_input_change_and_request_reinit()) break;
      bool got=false;
      (void)p.step_capture_publish_blocking(50, &got);
      (void)got;
    }
    std::vector<uint8_t> rgb;
    if (grab_one_frame_rgb24(p.vfd, p.cbufs, p.cbuf_count, p.in_stride, p.in_w, p.in_h, rgb)) {
      ref_png = rgb24_to_png_uncompressed(rgb.data(), p.in_w, p.in_h, p.in_w*3);
      fprintf(stderr, "[webui] reference PNG captured: %zu bytes\n", ref_png.size());
    } else {
      fprintf(stderr, "[webui] reference PNG capture failed\n");
    }
  }

  // WEBUI STARTUP
  if (webui_enabled) {
    fprintf(stderr, "[webui] wiring handlers...\n");
    webui_set_config_json_provider(&config_json_provider);
    webui_set_apply_handler(&apply_from_body);
    webui_set_status_provider(&status_json);
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
    if (!p.step_capture_publish_blocking(1000, &got_new)) {
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

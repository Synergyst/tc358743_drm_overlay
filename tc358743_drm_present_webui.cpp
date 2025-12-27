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

#include "tc358743_webui.h"

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
  req.count = 3; // keep 3; can experiment with 2 for latency later
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

static uint32_t get_plane_prop_value_u64(int drm_fd, uint32_t plane_id, uint32_t prop_id, bool *ok) {
  *ok=false;
  drmModeObjectProperties *props = drmModeObjectGetProperties(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE);
  if (!props) return 0;
  uint64_t val=0;
  for (uint32_t i=0;i<props->count_props;i++) {
    if (props->props[i] == prop_id) { val = props->prop_values[i]; *ok=true; break; }
  }
  drmModeFreeObjectProperties(props);
  return (uint32_t)val;
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
      uint32_t type_val = get_plane_prop_value_u64(drm_fd, p->plane_id, type_prop, &ok);
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

static bool parse_rect_csv(const std::string &s, rect_u32 &r) {
  if (s.empty()) return false;
  unsigned long x=0,y=0,w=0,h=0;
  if (sscanf(s.c_str(), "%lu,%lu,%lu,%lu", &x,&y,&w,&h) != 4) return false;
  if (w==0||h==0) return false;
  r.x=(uint32_t)x; r.y=(uint32_t)y; r.w=(uint32_t)w; r.h=(uint32_t)h;
  return true;
}

static bool parse_scale_csv(const std::string &s, float &sx, float &sy) {
  if (s.empty()) return false;
  double a=0,b=0;
  if (sscanf(s.c_str(), "%lf,%lf", &a, &b) != 2) return false;
  if (!(a>0.0001 && b>0.0001)) return false;
  sx=(float)a; sy=(float)b;
  return true;
}

static bool parse_pos_csv_str(const std::string &s, point_i32 &p) {
  if (s.empty()) return false;
  int x=0,y=0;
  if (sscanf(s.c_str(), "%d,%d", &x, &y) != 2) return false;
  p.x=x; p.y=y;
  return true;
}

static inline uint32_t src_read_xrgb8888(const uint8_t *src_rgb, uint32_t src_stride,
                                         uint32_t sx, uint32_t sy, bool input_is_bgr) {
  const uint8_t *p = src_rgb + (uint64_t)sy*src_stride + (uint64_t)sx*3ull;
  uint8_t r,g,b;
  if (!input_is_bgr) { r=p[0]; g=p[1]; b=p[2]; }
  else { b=p[0]; g=p[1]; r=p[2]; }
  return 0xFF000000u | ((uint32_t)r<<16) | ((uint32_t)g<<8) | (uint32_t)b;
}

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
};

struct viewport_cfg { bool set=false; uint32_t w=0,h=0; };

static std::string layer_invert_to_str(layer_invert_rel r) {
  if (r==INV_LOWER) return "lower";
  if (r==INV_UPPER) return "upper";
  return "none";
}
static layer_invert_rel layer_invert_from_str(const std::string &s) {
  if (s=="lower") return INV_LOWER;
  if (s=="upper") return INV_UPPER;
  return INV_NONE;
}
static std::string layer_type_to_str(layer_type t) {
  if (t==LAYER_CROSSHAIR) return "crosshair";
  if (t==LAYER_GRAPHICS) return "graphics";
  return "video";
}
static layer_type layer_type_from_str(const std::string &s) {
  if (s=="crosshair") return LAYER_CROSSHAIR;
  if (s=="graphics") return LAYER_GRAPHICS;
  return LAYER_VIDEO;
}

// Crosshair runtime + bounded draw
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
                                        const uint32_t *upper_buf /*nullable*/) {
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

// INV_UPPER cache (per-layer snapshots)
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

static void clear_xrgb_rows(uint8_t *dst_xrgb, uint32_t dst_stride, uint32_t out_w, uint32_t y0, uint32_t y1) {
  for (uint32_t y=y0;y<y1;y++) {
    uint32_t *d = (uint32_t*)(dst_xrgb + (uint64_t)y*dst_stride);
    for (uint32_t x=0;x<out_w;x++) d[x]=0xFF000000u;
  }
}

// ---------------- Compositor thread pool ----------------
struct comp_job {
  uint64_t job_id=0; // generation ID
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

// New mapping helpers (fast compositor)
static inline int32_t map_vx_to_ox(const present_rect &pr, uint32_t vpw, int32_t vx) {
  return pr.crtc_x + (int32_t)(((int64_t)vx * (int64_t)pr.crtc_w) / (int64_t)vpw);
}
static inline int32_t map_vy_to_oy(const present_rect &pr, uint32_t vph, int32_t vy) {
  return pr.crtc_y + (int32_t)(((int64_t)vy * (int64_t)pr.crtc_h) / (int64_t)vph);
}
static inline bool oy_to_vy_span(const present_rect &pr, uint32_t vph, int32_t oy, int32_t *vy0, int32_t *vy1_excl) {
  if (!vy0 || !vy1_excl) return false;
  if (pr.crtc_h == 0 || vph == 0) return false;
  int32_t t = oy - pr.crtc_y;
  if (t < 0 || t >= (int32_t)pr.crtc_h) return false;
  int64_t num0 = (int64_t)t * (int64_t)vph;
  int64_t num1 = (int64_t)(t + 1) * (int64_t)vph;
  int64_t den = (int64_t)pr.crtc_h;
  int32_t a = (int32_t)((num0 + den - 1) / den);
  int32_t b = (int32_t)((num1 + den - 1) / den);
  if (a < 0) a = 0;
  if (b < 0) b = 0;
  if (a > (int32_t)vph) a = (int32_t)vph;
  if (b > (int32_t)vph) b = (int32_t)vph;
  *vy0 = a;
  *vy1_excl = b;
  return (b > a);
}
static inline bool ox_to_vx_span(const present_rect &pr, uint32_t vpw, int32_t ox, int32_t *vx0, int32_t *vx1_excl) {
  if (!vx0 || !vx1_excl) return false;
  if (pr.crtc_w == 0 || vpw == 0) return false;
  int32_t t = ox - pr.crtc_x;
  if (t < 0 || t >= (int32_t)pr.crtc_w) return false;
  int64_t num0 = (int64_t)t * (int64_t)vpw;
  int64_t num1 = (int64_t)(t + 1) * (int64_t)vpw;
  int64_t den = (int64_t)pr.crtc_w;
  int32_t a = (int32_t)((num0 + den - 1) / den);
  int32_t b = (int32_t)((num1 + den - 1) / den);
  if (a < 0) a = 0;
  if (b < 0) b = 0;
  if (a > (int32_t)vpw) a = (int32_t)vpw;
  if (b > (int32_t)vpw) b = (int32_t)vpw;
  *vx0 = a;
  *vx1_excl = b;
  return (b > a);
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
  // expand by 1px safety
  ox0 -= 1; oy0 -= 1;
  ox1 += 1; oy1 += 1;
  r.x0 = ox0; r.y0 = oy0; r.x1 = ox1; r.y1 = oy1;
  return r;
}

static inline rect_i32 clamp_bounds_to_output_and_band(const rect_i32 &r, uint32_t out_w, uint32_t out_h, uint32_t y0, uint32_t y1) {
  rect_i32 out = r;
  rect_i32 out_full;
  out_full.x0 = 0;
  out_full.y0 = 0;
  out_full.x1 = (int32_t)out_w;
  out_full.y1 = (int32_t)out_h;
  rect_i32 band;
  band.x0 = 0;
  band.y0 = (int32_t)y0;
  band.x1 = (int32_t)out_w;
  band.y1 = (int32_t)y1;
  out = rect_intersect(out, out_full);
  out = rect_intersect(out, band);
  return out;
}

// FAST compositor
static void composite_layers_rows(const comp_job &job, uint32_t y0, uint32_t y1) {
  const auto &layers = *job.layers;

  clear_xrgb_rows(job.dst_xrgb, job.dst_stride, job.out_w, y0, y1);

  const present_rect &pr = job.map_pr;
  const uint32_t vpw = job.vpw ? job.vpw : 1;
  const uint32_t vph = job.vph ? job.vph : 1;

  for (size_t li = 0; li < layers.size(); li++) {
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

    rect_u32 sr = L.src_rect;
    if (sr.w == 0 || sr.h == 0) continue;
    if (sr.x >= job.in_w || sr.y >= job.in_h) continue;
    if (sr.x + sr.w > job.in_w) sr.w = job.in_w - sr.x;
    if (sr.y + sr.h > job.in_h) sr.h = job.in_h - sr.y;

    float sx = L.scale_x, sy = L.scale_y;
    if (!(sx > 0.0001f && sy > 0.0001f)) continue;

    const int32_t layer_vp_x0 = L.dst_pos.x;
    const int32_t layer_vp_y0 = L.dst_pos.y;
    const int32_t layer_vp_w  = (int32_t)std::max(1.0f, std::floor((float)sr.w * sx + 0.5f));
    const int32_t layer_vp_h  = (int32_t)std::max(1.0f, std::floor((float)sr.h * sy + 0.5f));

    uint8_t a = (uint8_t)std::lround(std::clamp(L.opacity, 0.0f, 1.0f) * 255.0f);
    if (a == 0) continue;

    rect_i32 ob = layer_vp_rect_to_output_bounds(pr, vpw, vph, layer_vp_x0, layer_vp_y0, layer_vp_w, layer_vp_h);
    ob = clamp_bounds_to_output_and_band(ob, job.out_w, job.out_h, y0, y1);
    if (rect_empty(ob)) continue;

    for (int32_t oy = ob.y0; oy < ob.y1; oy++) {
      int32_t vy_a = 0, vy_b = 0;
      if (!oy_to_vy_span(pr, vph, oy, &vy_a, &vy_b)) continue;

      const int32_t layer_vp_y1 = layer_vp_y0 + layer_vp_h;
      int32_t vy0 = std::max(vy_a, layer_vp_y0);
      int32_t vy1 = std::min(vy_b, layer_vp_y1);
      if (vy0 >= vy1) continue;

      // sample top of span (fast, deterministic)
      int32_t vyy = vy0;

      float local_y = (float)(vyy - layer_vp_y0);
      uint32_t syi = sr.y + (uint32_t)std::min((uint32_t)sr.h - 1u, (uint32_t)(local_y / sy));

      uint32_t *drow = (uint32_t*)(job.dst_xrgb + (uint64_t)oy * job.dst_stride);
      const uint32_t *urow = upper_buf ? (upper_buf + (size_t)oy * (size_t)job.out_w) : nullptr;

      for (int32_t ox = ob.x0; ox < ob.x1; ox++) {
        int32_t vx_a = 0, vx_b = 0;
        if (!ox_to_vx_span(pr, vpw, ox, &vx_a, &vx_b)) continue;

        const int32_t layer_vp_x1 = layer_vp_x0 + layer_vp_w;
        int32_t vx0 = std::max(vx_a, layer_vp_x0);
        int32_t vx1 = std::min(vx_b, layer_vp_x1);
        if (vx0 >= vx1) continue;

        // sample left of span (fast, deterministic)
        int32_t vxx = vx0;

        float local_x = (float)(vxx - layer_vp_x0);
        uint32_t sxi = sr.x + (uint32_t)std::min((uint32_t)sr.w - 1u, (uint32_t)(local_x / sx));

        uint32_t over = src_read_xrgb8888(job.src_rgb, job.src_stride, sxi, syi, job.input_is_bgr);
        if (L.invert_rel == INV_LOWER) {
          over = invert_xrgb8888(drow[(uint32_t)ox]);
        } else if (L.invert_rel == INV_UPPER) {
          uint32_t base = urow ? urow[(uint32_t)ox] : 0xFF000000u;
          over = invert_xrgb8888(base);
        }
        drow[(uint32_t)ox] = blend_over_xrgb(drow[(uint32_t)ox], over, a);
      }
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

// ---------------- Minimal JSON helpers (unchanged) ----------------
static std::string json_escape(const std::string &s) {
  std::string o; o.reserve(s.size()+8);
  for (char c: s) {
    switch (c) {
      case '\\': o += "\\\\"; break;
      case '"':  o += "\\\""; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default: o += c; break;
    }
  }
  return o;
}

static bool json_get_string(const std::string &j, const std::string &key, std::string &out) {
  std::string pat = "\"" + key + "\"";
  auto k = j.find(pat);
  if (k == std::string::npos) return false;
  auto colon = j.find(':', k + pat.size());
  if (colon == std::string::npos) return false;
  auto q1 = j.find('"', colon + 1);
  if (q1 == std::string::npos) return false;
  auto q2 = j.find('"', q1 + 1);
  if (q2 == std::string::npos) return false;
  out = j.substr(q1 + 1, q2 - (q1 + 1));
  return true;
}

static bool json_get_bool(const std::string &j, const std::string &key, bool &out) {
  std::string pat = "\"" + key + "\"";
  auto k = j.find(pat);
  if (k == std::string::npos) return false;
  auto colon = j.find(':', k + pat.size());
  if (colon == std::string::npos) return false;
  auto v = j.substr(colon + 1);
  auto first = v.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return false;
  v = v.substr(first);
  if (v.rfind("true", 0) == 0) { out=true; return true; }
  if (v.rfind("false",0) == 0) { out=false; return true; }
  return false;
}

static bool json_get_int(const std::string &j, const std::string &key, int &out) {
  std::string pat = "\"" + key + "\"";
  auto k = j.find(pat);
  if (k == std::string::npos) return false;
  auto colon = j.find(':', k + pat.size());
  if (colon == std::string::npos) return false;
  auto v = j.substr(colon + 1);
  auto first = v.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return false;
  size_t i = first;
  if (v[i] == '-') i++;
  size_t start = i;
  while (i < v.size() && (v[i] >= '0' && v[i] <= '9')) i++;
  if (i == start) return false;
  out = atoi(v.substr(first, i-first).c_str());
  return true;
}

static bool json_get_double(const std::string &j, const std::string &key, double &out) {
  std::string pat = "\"" + key + "\"";
  auto k = j.find(pat);
  if (k == std::string::npos) return false;
  auto colon = j.find(':', k + pat.size());
  if (colon == std::string::npos) return false;
  auto v = j.substr(colon + 1);
  auto first = v.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return false;
  char *end=nullptr;
  out = strtod(v.c_str() + first, &end);
  if (end == (v.c_str()+first)) return false;
  return true;
}

static bool json_get_array_object_slices(const std::string &j, const std::string &key, std::vector<std::string> &out_objs) {
  out_objs.clear();
  std::string pat="\"" + key + "\"";
  auto k=j.find(pat);
  if (k==std::string::npos) return false;
  auto colon=j.find(':', k+pat.size());
  if (colon==std::string::npos) return false;
  auto lb=j.find('[', colon+1);
  if (lb==std::string::npos) return false;
  size_t i=lb+1;
  int depth=0;
  size_t start=std::string::npos;
  for (; i<j.size(); i++) {
    char c=j[i];
    if (c=='{') { if (depth==0) start=i; depth++; }
    else if (c=='}') {
      if (depth>0) depth--;
      if (depth==0 && start!=std::string::npos) {
        out_objs.push_back(j.substr(start, i-start+1));
        start=std::string::npos;
      }
    } else if (c==']' && depth==0) return true;
  }
  return true;
}

// ---------------- Config structure ----------------
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

std::string config_to_json(const persisted_config &c) {
  std::ostringstream ss;
  ss << "{";
  ss << "\"v4l2Dev\":\"" << json_escape(c.v4l2_dev) << "\",";
  ss << "\"edidPath\":\"" << json_escape(c.edid_path) << "\",";
  ss << "\"modesetMatchInput\":" << (c.do_modeset ? "true":"false") << ",";
  ss << "\"threads\":" << c.threads << ",";
  ss << "\"bgr\":" << (c.input_is_bgr ? "true":"false") << ",";
  ss << "\"present\":\"" << json_escape(c.present_policy) << "\",";
  ss << "\"preferOutputMode\":" << (c.prefer_output_mode ? "true":"false") << ",";
  ss << "\"preferOutputRes\":\"" << c.prefer_out_w << "x" << c.prefer_out_h << "\",";
  ss << "\"viewport\":\"";
  if (c.viewport.set) ss << c.viewport.w << "x" << c.viewport.h;
  ss << "\",";
  ss << "\"listenAddr\":\"" << json_escape(c.listen_addr) << "\",";
  ss << "\"layers\":[";
  for (size_t i=0;i<c.layers.size();i++) {
    const auto &L=c.layers[i];
    if (i) ss << ",";
    ss << "{";
    ss << "\"name\":\"" << json_escape(L.name) << "\",";
    ss << "\"type\":\"" << json_escape(layer_type_to_str(L.type)) << "\",";
    ss << "\"enabled\":" << (L.enabled?"true":"false") << ",";
    ss << "\"opacity\":" << std::clamp(L.opacity, 0.0f, 1.0f) << ",";
    ss << "\"invertRel\":\"" << json_escape(layer_invert_to_str(L.invert_rel)) << "\",";
    ss << "\"srcRect\":\"" << L.src_rect.x << "," << L.src_rect.y << "," << L.src_rect.w << "," << L.src_rect.h << "\",";
    ss << "\"dstPos\":\"" << L.dst_pos.x << "," << L.dst_pos.y << "\",";
    ss << "\"scale\":\"" << L.scale_x << "," << L.scale_y << "\",";
    ss << "\"crosshair\":{";
    ss << "\"enabled\":" << (L.xh.enabled?"true":"false") << ",";
    ss << "\"diam\":\"" << L.xh.diam_w << "x" << L.xh.diam_h << "\",";
    ss << "\"center\":\"" << (L.xh.center_set ? (std::to_string(L.xh.cx)+","+std::to_string(L.xh.cy)) : "") << "\",";
    ss << "\"thickness\":" << L.xh.thickness << ",";
    ss << "\"mode\":\"" << (L.xh.solid?"solid":"invert") << "\",";
    ss << "\"color\":\"" << (int)L.xh.r << "," << (int)L.xh.g << "," << (int)L.xh.b << "\",";
    ss << "\"opacity\":" << std::clamp(L.xh.opacity, 0.0f, 1.0f) << ",";
    ss << "\"invertRel\":\"" << json_escape(layer_invert_to_str(L.xh.invert_rel)) << "\"";
    ss << "}";
    ss << "}";
  }
  ss << "]";
  ss << "}";
  return ss.str();
}

static bool config_from_json(const std::string &j, persisted_config &c) {
  std::string s; bool b; int n; double d;
  if (json_get_string(j, "v4l2Dev", s)) c.v4l2_dev=s;
  if (json_get_string(j, "edidPath", s)) c.edid_path=s;
  if (json_get_bool(j, "modesetMatchInput", b)) c.do_modeset=b;
  if (json_get_int(j, "threads", n)) c.threads=n;
  if (json_get_bool(j, "bgr", b)) c.input_is_bgr=b;
  if (json_get_string(j, "present", s) && present_policy_valid(s)) c.present_policy=s;
  if (json_get_bool(j, "preferOutputMode", b)) c.prefer_output_mode=b;
  if (json_get_string(j, "preferOutputRes", s)) {
    uint32_t w=0,h=0; if (parse_dim(s.c_str(), &w,&h)) { c.prefer_out_w=w; c.prefer_out_h=h; }
  }
  if (json_get_string(j, "viewport", s)) {
    if (s.empty()) c.viewport.set=false;
    else {
      uint32_t w=0,h=0;
      if (parse_dim(s.c_str(), &w,&h)) { c.viewport.set=true; c.viewport.w=w; c.viewport.h=h; }
    }
  }
  if (json_get_string(j, "listenAddr", s) && !s.empty()) c.listen_addr=s;

  std::vector<std::string> layer_objs;
  if (json_get_array_object_slices(j, "layers", layer_objs)) {
    std::vector<layer_cfg> parsed;
    for (auto &obj : layer_objs) {
      layer_cfg L;
      if (json_get_string(obj, "name", s)) L.name=s;
      if (json_get_string(obj, "type", s)) L.type=layer_type_from_str(s);
      if (json_get_bool(obj, "enabled", b)) L.enabled=b;
      if (json_get_double(obj, "opacity", d)) { if (d<0)d=0; if (d>1)d=1; L.opacity=(float)d; }
      if (json_get_string(obj, "invertRel", s)) L.invert_rel=layer_invert_from_str(s);
      if (json_get_string(obj, "srcRect", s)) { rect_u32 r; if (parse_rect_csv(s,r)) L.src_rect=r; }
      if (json_get_string(obj, "dstPos", s)) { point_i32 p; if (parse_pos_csv_str(s,p)) L.dst_pos=p; }
      if (json_get_string(obj, "scale", s)) { float sx=1,sy=1; if (parse_scale_csv(s,sx,sy)) { L.scale_x=sx; L.scale_y=sy; } }

      std::string xhobj;
      std::string pat="\"crosshair\"";
      auto k=obj.find(pat);
      if (k!=std::string::npos) {
        auto lb=obj.find('{', k+pat.size());
        if (lb!=std::string::npos) {
          int depth=0; size_t start=lb;
          for (size_t i=lb;i<obj.size();i++){
            if (obj[i]=='{') depth++;
            else if (obj[i]=='}'){ depth--; if(depth==0){ xhobj=obj.substr(start, i-start+1); break; } }
          }
        }
      }
      if (!xhobj.empty()) {
        if (json_get_bool(xhobj, "enabled", b)) L.xh.enabled=b;
        if (json_get_string(xhobj, "diam", s)) { uint32_t w=0,h=0; if (parse_dim(s.c_str(), &w,&h)) { L.xh.diam_w=w; L.xh.diam_h=h; } }
        if (json_get_string(xhobj, "center", s)) {
          if (s.empty()) L.xh.center_set=false;
          else { int32_t cx=0,cy=0; if (parse_point_csv(s.c_str(), &cx,&cy)) { L.xh.center_set=true; L.xh.cx=cx; L.xh.cy=cy; } }
        }
        if (json_get_int(xhobj, "thickness", n)) { if (n>=1 && n<=99) L.xh.thickness=(uint32_t)n; }
        if (json_get_string(xhobj, "mode", s)) L.xh.solid=(s!="invert");
        if (json_get_string(xhobj, "color", s)) { uint8_t rr=0,gg=0,bb=0; if (parse_rgb_csv(s.c_str(), &rr,&gg,&bb)) { L.xh.r=rr; L.xh.g=gg; L.xh.b=bb; } }
        if (json_get_double(xhobj, "opacity", d)) { if (d<0)d=0; if (d>1)d=1; L.xh.opacity=(float)d; }
        if (json_get_string(xhobj, "invertRel", s)) L.xh.invert_rel=layer_invert_from_str(s);
      }

      parsed.push_back(L);
    }
    c.layers = parsed;
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
  z.push_back(0x78); z.push_back(0x01); // zlib header (no compression)
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
  // held capture buffer (pipeline-owned)
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

  // stable storage for worker threads
  std::vector<layer_cfg> layers_work_;

  // seq gating
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
    if (!p_fb_id || !p_crtc_id || !p_crtc_x || !p_crtc_y || !p_crtc_w || !p_crtc_h ||
        !p_src_x || !p_src_y || !p_src_w || !p_src_h) return false;

    present_rect scanout{};
    scanout.crtc_x=0; scanout.crtc_y=0;
    scanout.crtc_w=out_w; scanout.crtc_h=out_h;
    scanout.src_w=out_w; scanout.src_h=out_h;

    if (dumb_fb_create(drm_fd, out_w, out_h, fmt, &fb[0]) != 0) return false;
    if (dumb_fb_create(drm_fd, out_w, out_h, fmt, &fb[1]) != 0) return false;

    // initial test pattern (kept)
    {
      uint32_t *d=(uint32_t*)fb[0].map;
      uint32_t stride_u32 = fb[0].pitch/4;
      for (uint32_t y=0;y<out_h;y++) for (uint32_t x=0;x<out_w;x++) {
        uint32_t r=(x*255)/(out_w?out_w:1);
        uint32_t g=(y*255)/(out_h?out_h:1);
        d[y*stride_u32+x] = 0xFF000000u | (r<<16) | (g<<8) | 0x40;
      }
    }

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

    // NOTE: keep held buffer state local; STREAMOFF returns buffers anyway on most drivers
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

  // Low-latency: block until at least one frame is ready (or timeout for supervisor housekeeping).
  // Returns: false on fatal error, true otherwise. Sets *got_new_frame = true only if seq advanced.
  bool step_capture_publish_blocking(int timeout_ms, bool *got_new_frame) {
    if (got_new_frame) *got_new_frame = false;

    uint64_t before_seq = g_latest_seq.load(std::memory_order_acquire);

    int prc = v4l2_wait_for_frame(vfd, timeout_ms);
    if (prc < 0) return false;
    if (prc == 0) return true; // timeout, no frame

    struct v4l2_buffer b;
    int dq=v4l2_dequeue_latest_frame(vfd, &b);
    if (dq==1) return true;
    if (dq<0) return false;

    const uint8_t *src = (const uint8_t*)cbufs[b.index].start;
    if (!dumped) { dump_first_pixels_rgb24(src); dumped=true; }

    // Re-queue previous held buffer, hold newest
    if (have_held_buf) {
      (void)v4l2_queue_frame(vfd, &held_buf);
    }
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

  // Helper: build upper_cache.running_upper using same mapping logic as compositor
  void build_upper_cache_fast(const latest_frame &lf, const present_rect &map_pr, uint32_t vpw, uint32_t vph, const persisted_config &cfg_live) {
    upper_cache.enabled = true;
    std::fill(upper_cache.running_upper.begin(), upper_cache.running_upper.end(), 0xFF000000u);
    upper_cache.clear_upper_of_layer_buffers();

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

      rect_u32 sr=L.src_rect;
      if (sr.w==0||sr.h==0) continue;
      if (sr.x>=lf.w||sr.y>=lf.h) continue;
      if (sr.x+sr.w>lf.w) sr.w=lf.w-sr.x;
      if (sr.y+sr.h>lf.h) sr.h=lf.h-sr.y;

      float sx=L.scale_x, sy=L.scale_y;
      if (!(sx>0.0001f && sy>0.0001f)) continue;

      uint8_t a=(uint8_t)std::lround(std::clamp(L.opacity,0.0f,1.0f)*255.0f);
      if (a==0) continue;

      const int32_t layer_vp_x0 = L.dst_pos.x;
      const int32_t layer_vp_y0 = L.dst_pos.y;
      const int32_t layer_vp_w  = (int32_t)std::max(1.0f, std::floor((float)sr.w * sx + 0.5f));
      const int32_t layer_vp_h  = (int32_t)std::max(1.0f, std::floor((float)sr.h * sy + 0.5f));

      rect_i32 ob = layer_vp_rect_to_output_bounds(map_pr, vpw, vph, layer_vp_x0, layer_vp_y0, layer_vp_w, layer_vp_h);
      ob = clamp_bounds_to_output_and_band(ob, out_w, out_h, 0, out_h);
      if (rect_empty(ob)) continue;

      for (int32_t oy = ob.y0; oy < ob.y1; oy++) {
        int32_t vy_a=0, vy_b=0;
        if (!oy_to_vy_span(map_pr, vph, oy, &vy_a, &vy_b)) continue;

        const int32_t layer_vp_y1 = layer_vp_y0 + layer_vp_h;
        int32_t vy0 = std::max(vy_a, layer_vp_y0);
        int32_t vy1 = std::min(vy_b, layer_vp_y1);
        if (vy0 >= vy1) continue;

        int32_t vyy = vy0;

        float local_y = (float)(vyy - layer_vp_y0);
        uint32_t syi = sr.y + (uint32_t)std::min((uint32_t)sr.h - 1u, (uint32_t)(local_y / sy));

        uint32_t *dstrow = &upper_cache.running_upper[(size_t)oy*(size_t)out_w];

        for (int32_t ox = ob.x0; ox < ob.x1; ox++) {
          int32_t vx_a=0, vx_b=0;
          if (!ox_to_vx_span(map_pr, vpw, ox, &vx_a, &vx_b)) continue;

          const int32_t layer_vp_x1 = layer_vp_x0 + layer_vp_w;
          int32_t vx0 = std::max(vx_a, layer_vp_x0);
          int32_t vx1 = std::min(vx_b, layer_vp_x1);
          if (vx0 >= vx1) continue;

          int32_t vxx = vx0;

          float local_x = (float)(vxx - layer_vp_x0);
          uint32_t sxi = sr.x + (uint32_t)std::min((uint32_t)sr.w - 1u, (uint32_t)(local_x / sx));

          uint32_t over = src_read_xrgb8888(lf.ptr, lf.stride, sxi, syi, cfg_live.input_is_bgr);
          dstrow[(uint32_t)ox] = blend_over_xrgb(dstrow[(uint32_t)ox], over, a);
        }
      }
    }
  }

  // Render+present only when new frame arrives (seq gating)
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

  std::ostringstream ss;
  ss << "{";
  ss << "\"pid\":" << (int)g_pid.load() << ",";
  ss << "\"reinitRequested\":" << (g_reinit_requested.load() ? "true":"false") << ",";
  ss << "\"reinitReason\":\"" << json_escape(take_reinit_reason()) << "\",";
  ss << "\"configPath\":\"" << json_escape(g_cfg_path) << "\",";
  ss << "\"lastApplyOk\":" << (ok ? "true":"false") << ",";
  ss << "\"lastApplyError\":\"" << json_escape(err) << "\",";
  ss << "\"runtime\":{";
  ss << "\"outW\":" << outw << ",";
  ss << "\"outH\":" << outh << ",";
  ss << "\"vpw\":" << vpw << ",";
  ss << "\"vph\":" << vph << ",";
  ss << "\"presentPolicy\":\"" << json_escape(policy) << "\",";
  ss << "\"presentRect\":{";
  ss << "\"crtcX\":" << pr.crtc_x << ",";
  ss << "\"crtcY\":" << pr.crtc_y << ",";
  ss << "\"crtcW\":" << pr.crtc_w << ",";
  ss << "\"crtcH\":" << pr.crtc_h << ",";
  ss << "\"srcW\":" << pr.src_w << ",";
  ss << "\"srcH\":" << pr.src_h;
  ss << "}";
  ss << "}";
  ss << "}";
  return ss.str();
}

static bool apply_from_body(const std::string &body, std::string &out_json, int &out_http_status) {
  persisted_config cur = cfg_snapshot();
  persisted_config next = cur;
  std::string s; bool b; int n; double d;

  if (json_get_string(body, "v4l2Dev", s) && !s.empty()) next.v4l2_dev=s;
  if (json_get_string(body, "edidPath", s)) next.edid_path=s;
  if (json_get_bool(body, "modesetMatchInput", b)) next.do_modeset=b;
  if (json_get_int(body, "threads", n)) next.threads=n;
  if (json_get_bool(body, "bgr", b)) next.input_is_bgr=b;

  if (json_get_string(body, "present", s)) {
    if (!present_policy_valid(s)) { set_last_apply(false,"invalid present"); out_http_status=400; out_json="{\"error\":\"invalid present\"}"; return false; }
    next.present_policy=s;
  }

  if (json_get_bool(body, "preferOutputMode", b)) next.prefer_output_mode=b;

  if (json_get_string(body, "preferOutputRes", s)) {
    if (!s.empty()) {
      uint32_t w=0,h=0;
      if (!parse_dim(s.c_str(), &w,&h)) { set_last_apply(false,"invalid preferOutputRes"); out_http_status=400; out_json="{\"error\":\"invalid preferOutputRes\"}"; return false; }
      next.prefer_out_w=w; next.prefer_out_h=h;
    }
  }

  if (json_get_string(body, "viewport", s)) {
    if (s.empty()) next.viewport.set=false;
    else {
      uint32_t w=0,h=0;
      if (!parse_dim(s.c_str(), &w,&h)) { set_last_apply(false,"invalid viewport"); out_http_status=400; out_json="{\"error\":\"invalid viewport\"}"; return false; }
      next.viewport.set=true; next.viewport.w=w; next.viewport.h=h;
    }
  }

  if (json_get_string(body, "listenAddr", s) && !s.empty()) next.listen_addr=s;

  std::vector<std::string> layer_objs;
  if (json_get_array_object_slices(body, "layers", layer_objs)) {
    std::vector<layer_cfg> parsed;
    for (auto &obj : layer_objs) {
      layer_cfg L;
      if (json_get_string(obj, "name", s)) L.name=s;
      if (json_get_string(obj, "type", s)) L.type=layer_type_from_str(s);
      if (json_get_bool(obj, "enabled", b)) L.enabled=b;
      if (json_get_double(obj, "opacity", d)) { if(d<0||d>1){ out_http_status=400; out_json="{\"error\":\"invalid layer opacity\"}"; return false; } L.opacity=(float)d; }
      if (json_get_string(obj, "invertRel", s)) { if(!(s=="none"||s=="lower"||s=="upper")){ out_http_status=400; out_json="{\"error\":\"invalid layer invertRel\"}"; return false; } L.invert_rel=layer_invert_from_str(s); }
      if (json_get_string(obj, "srcRect", s)) { rect_u32 r; if(!parse_rect_csv(s,r)){ out_http_status=400; out_json="{\"error\":\"invalid layer srcRect\"}"; return false; } L.src_rect=r; }
      if (json_get_string(obj, "dstPos", s)) { point_i32 p; if(!parse_pos_csv_str(s,p)){ out_http_status=400; out_json="{\"error\":\"invalid layer dstPos\"}"; return false; } L.dst_pos=p; }
      if (json_get_string(obj, "scale", s)) { float sx=1,sy=1; if(!parse_scale_csv(s,sx,sy)){ out_http_status=400; out_json="{\"error\":\"invalid layer scale\"}"; return false; } L.scale_x=sx; L.scale_y=sy; }

      std::string xhobj;
      std::string pat="\"crosshair\"";
      auto k=obj.find(pat);
      if (k!=std::string::npos) {
        auto lb=obj.find('{', k+pat.size());
        if (lb!=std::string::npos) {
          int depth=0; size_t start=lb;
          for (size_t i=lb;i<obj.size();i++){
            if (obj[i]=='{') depth++;
            else if (obj[i]=='}'){ depth--; if(depth==0){ xhobj=obj.substr(start, i-start+1); break; } }
          }
        }
      }
      if (!xhobj.empty()) {
        if (json_get_bool(xhobj,"enabled",b)) L.xh.enabled=b;
        if (json_get_string(xhobj,"diam",s)) { uint32_t w=0,h=0; if(!parse_dim(s.c_str(),&w,&h)){ out_http_status=400; out_json="{\"error\":\"invalid crosshair diam\"}"; return false; } L.xh.diam_w=w; L.xh.diam_h=h; }
        if (json_get_string(xhobj,"center",s)) {
          if (s.empty()) L.xh.center_set=false;
          else { int32_t cx=0,cy=0; if(!parse_point_csv(s.c_str(),&cx,&cy)){ out_http_status=400; out_json="{\"error\":\"invalid crosshair center\"}"; return false; } L.xh.center_set=true; L.xh.cx=cx; L.xh.cy=cy; }
        }
        if (json_get_int(xhobj,"thickness",n)) { if(n<1||n>99){ out_http_status=400; out_json="{\"error\":\"invalid crosshair thickness\"}"; return false; } L.xh.thickness=(uint32_t)n; }
        if (json_get_string(xhobj,"mode",s)) { if(!(s=="solid"||s=="invert")){ out_http_status=400; out_json="{\"error\":\"invalid crosshair mode\"}"; return false; } L.xh.solid=(s=="solid"); }
        if (json_get_string(xhobj,"color",s)) { uint8_t rr=0,gg=0,bb=0; if(!parse_rgb_csv(s.c_str(),&rr,&gg,&bb)){ out_http_status=400; out_json="{\"error\":\"invalid crosshair color\"}"; return false; } L.xh.r=rr; L.xh.g=gg; L.xh.b=bb; }
        if (json_get_double(xhobj,"opacity",d)) { if(d<0||d>1){ out_http_status=400; out_json="{\"error\":\"invalid crosshair opacity\"}"; return false; } L.xh.opacity=(float)d; }
        if (json_get_string(xhobj,"invertRel",s)) { if(!(s=="none"||s=="lower"||s=="upper")){ out_http_status=400; out_json="{\"error\":\"invalid crosshair invertRel\"}"; return false; } L.xh.invert_rel=layer_invert_from_str(s); }
      }

      parsed.push_back(L);
    }
    next.layers = parsed;
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

  std::ostringstream out;
  out << "{";
  out << "\"ok\":true,";
  out << "\"saved\":true,";
  out << "\"reinitRequested\":" << (need_reinit?"true":"false") << ",";
  out << "\"reinitReason\":\"" << json_escape(reason) << "\",";
  out << "\"effectiveConfig\":" << config_to_json(next);
  out << "}";
  out_json=out.str();
  out_http_status=200;
  return true;
}

static std::string config_json_provider() {
  return config_to_json(cfg_snapshot());
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
    std::string j = slurp_file(g_cfg_path);
    if (!j.empty()) {
      (void)config_from_json(j, loaded);
      fprintf(stderr, "[config] loaded %s\n", g_cfg_path.c_str());
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

  // init pipeline supervisor
  for (;;) {
    auto snap=cfg_snapshot();
    if (!present_policy_valid(snap.present_policy)) snap.present_policy="fit";
    if (p.init_from_config(snap)) { have_pipeline=true; break; }
    fprintf(stderr, "[supervisor] pipeline init failed; retrying in 1s...\n");
    p.shutdown();
    usleep(1000*1000);
  }

  // capture reference png (unchanged)
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

  if (webui_enabled) {
    webui_set_config_json_provider(&config_json_provider);
    webui_set_apply_handler(&apply_from_body);
    webui_set_status_provider(&status_json);
    webui_set_quit_flag(&g_quit);
    webui_set_reference_png(ref_png);
    auto snap=cfg_snapshot();
    webui_set_listen_address(snap.listen_addr);
    webui_start_detached(webui_port);
    fprintf(stderr, "[webui] open http://<device-ip>:%d/\n", webui_port);
  }

  // Main loop: frame-driven, no sleeps, render only when seq changes
  while (!g_quit.load()) {
    // quit by keyboard
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

    // BLOCK until at least one frame arrives (timeout just to service reinit/quit frequently)
    bool got_new=false;
    if (!p.step_capture_publish_blocking(1000, &got_new)) {
      fprintf(stderr, "[supervisor] capture failed; reinit...\n");
      p.shutdown(); have_pipeline=false;
      continue;
    }

    // If we timed out and got no frame, loop again (keep checking reinit/quit)
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

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

#if defined(__aarch64__) || defined(__ARM_NEON)
  #include <arm_neon.h>
  #define HAVE_NEON 1
#else
  #define HAVE_NEON 0
#endif

#include "third_party/httplib.h"

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
  *out_buf = NULL;
  *out_len = 0;
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
  *out_buf = buf;
  *out_len = (size_t)len;
  return true;
}

static void fix_edid_checksums_inplace(uint8_t *edid, size_t len) {
  if (len % 128 != 0) return;
  size_t blocks = len / 128;
  for (size_t b = 0; b < blocks; b++) {
    uint8_t sum = 0;
    uint8_t *blk = edid + b * 128;
    for (int i = 0; i < 127; i++) sum = (uint8_t)(sum + blk[i]);
    blk[127] = (uint8_t)(0x100 - sum);
  }
}

static int v4l2_set_edid_if_requested(int fd, const char *edid_path) {
  if (!edid_path || !edid_path[0]) return 0;
  uint8_t *edid = NULL;
  size_t len = 0;
  if (!read_file_binary(edid_path, &edid, &len)) {
    fprintf(stderr, "[v4l2] Failed to read EDID: %s\n", edid_path);
    return -1;
  }
  if (len < 128 || (len % 128) != 0) {
    fprintf(stderr, "[v4l2] Invalid EDID size: %zu\n", len);
    free(edid);
    return -1;
  }
  fix_edid_checksums_inplace(edid, len);
  struct v4l2_edid vedid;
  memset(&vedid, 0, sizeof(vedid));
  vedid.start_block = 0;
  vedid.blocks = (uint32_t)(len / 128);
  vedid.edid = edid;
  if (xioctl(fd, VIDIOC_S_EDID, &vedid) < 0) {
    fprintf(stderr, "[v4l2] VIDIOC_S_EDID failed: %s\n", strerror(errno));
    free(edid);
    return -1;
  }
  fprintf(stderr, "[v4l2] EDID set (%u blocks) from %s\n", vedid.blocks, edid_path);
  free(edid);
  return 0;
}

static int tc358743_query_and_set_dv_timings(int fd, uint32_t *out_w, uint32_t *out_h, uint64_t *out_pixelclock) {
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
  fprintf(stderr, "[v4l2] DV timings: %ux%u pixelclock=%llu\n",
          t.bt.width, t.bt.height, (unsigned long long)t.bt.pixelclock);
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
  uint32_t fourcc = fmt.fmt.pix.pixelformat;
  char fcc[5] = {
    (char)(fourcc & 0xFF),
    (char)((fourcc >> 8) & 0xFF),
    (char)((fourcc >> 16) & 0xFF),
    (char)((fourcc >> 24) & 0xFF),
    0
  };
  fprintf(stderr, "[v4l2] Format: %ux%u fourcc=%s bytesperline=%u sizeimage=%u\n",
          fmt.fmt.pix.width, fmt.fmt.pix.height, fcc,
          fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
  return 0;
}

static int v4l2_start_mmap_capture(int fd, struct cap_buf **out_bufs, uint32_t *out_nbufs) {
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 2;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    fprintf(stderr, "[v4l2] VIDIOC_REQBUFS failed: %s\n", strerror(errno));
    return -1;
  }
  if (req.count < 2) {
    fprintf(stderr, "[v4l2] Not enough buffers (%u)\n", req.count);
    return -1;
  }
  struct cap_buf *bufs = (struct cap_buf*)calloc(req.count, sizeof(*bufs));
  if (!bufs) return -1;
  for (uint32_t i = 0; i < req.count; i++) {
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
  for (uint32_t i = 0; i < cbuf_count; i++) {
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
    fprintf(stderr, "[v4l2] poll failed: %s\n", strerror(errno));
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
    fprintf(stderr, "[v4l2] DQBUF failed: %s\n", strerror(errno));
    return -1;
  }
  *out_b = b;
  return 0;
}

static int v4l2_queue_frame(int fd, struct v4l2_buffer *b) {
  if (xioctl(fd, VIDIOC_QBUF, b) < 0) {
    fprintf(stderr, "[v4l2] QBUF failed: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

static int v4l2_dequeue_latest_frame(int fd, struct v4l2_buffer *out_b) {
  struct v4l2_buffer newest;
  bool have = false;
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
  fprintf(stderr, "[v4l2] first pixels (byte triples) @ (0,0..7): ");
  for (int i = 0; i < 8; i++) {
    const uint8_t *p = src + i * 3;
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
  out->width = w;
  out->height = h;
  out->format = format;

  struct drm_mode_create_dumb creq;
  memset(&creq, 0, sizeof(creq));
  creq.width = w;
  creq.height = h;
  creq.bpp = 32;

  if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) {
    perror("DRM_IOCTL_MODE_CREATE_DUMB");
    return -1;
  }
  out->handle = creq.handle;
  out->pitch = creq.pitch;
  out->size = creq.size;

  uint32_t handles[4] = { out->handle, 0, 0, 0 };
  uint32_t pitches[4] = { out->pitch, 0, 0, 0 };
  uint32_t offsets[4] = { 0, 0, 0, 0 };

  int ret = drmModeAddFB2(drm_fd, w, h, format, handles, pitches, offsets, &out->fb_id, 0);
  if (ret) {
    ret = drmModeAddFB(drm_fd, w, h, 24, 32, out->pitch, out->handle, &out->fb_id);
    if (ret) {
      perror("drmModeAddFB2/AddFB");
      return -1;
    }
  }

  struct drm_mode_map_dumb mreq;
  memset(&mreq, 0, sizeof(mreq));
  mreq.handle = out->handle;
  if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) {
    perror("DRM_IOCTL_MODE_MAP_DUMB");
    return -1;
  }

  out->map = mmap(0, out->size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, mreq.offset);
  if (out->map == MAP_FAILED) {
    perror("mmap dumb");
    out->map = NULL;
    return -1;
  }
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
  for (uint32_t i = 0; i < props->count_props; i++) {
    drmModePropertyRes *p = drmModeGetProperty(drm_fd, props->props[i]);
    if (p) {
      if (strcmp(p->name, name) == 0) {
        prop_id = p->prop_id;
        drmModeFreeProperty(p);
        break;
      }
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
  for (uint32_t i = 0; i < p->count_formats; i++) if (p->formats[i] == fmt) return true;
  return false;
}

static uint32_t get_plane_prop_value_u64(int drm_fd, uint32_t plane_id, uint32_t prop_id, bool *ok) {
  *ok = false;
  drmModeObjectProperties *props = drmModeObjectGetProperties(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE);
  if (!props) return 0;
  uint64_t val = 0;
  for (uint32_t i = 0; i < props->count_props; i++) {
    if (props->props[i] == prop_id) { val = props->prop_values[i]; *ok = true; break; }
  }
  drmModeFreeObjectProperties(props);
  return (uint32_t)val;
}

static uint32_t find_primary_plane_on_crtc(int drm_fd, uint32_t crtc_index, uint32_t format) {
  drmModePlaneRes *pres = drmModeGetPlaneResources(drm_fd);
  if (!pres) return 0;
  for (uint32_t i = 0; i < pres->count_planes; i++) {
    drmModePlane *p = drmModeGetPlane(drm_fd, pres->planes[i]);
    if (!p) continue;
    if (!(p->possible_crtcs & (1 << crtc_index))) { drmModeFreePlane(p); continue; }
    if (!plane_supports_format(p, format)) { drmModeFreePlane(p); continue; }
    uint32_t type_prop = get_prop_id(drm_fd, p->plane_id, DRM_MODE_OBJECT_PLANE, "type");
    if (type_prop) {
      bool ok = false;
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
  for (int i = 0; i < res->count_connectors; i++) {
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
  uint32_t chosen_crtc = 0, chosen_index = 0;
  uint32_t chosen_w = 0, chosen_h = 0;
  for (int i = 0; i < res->count_crtcs; i++) {
    if (!(enc->possible_crtcs & (1 << i))) continue;
    uint32_t cid = res->crtcs[i];
    drmModeCrtc *crtc = drmModeGetCrtc(drm_fd, cid);
    if (!crtc) continue;
    if (crtc->mode_valid && crtc->mode.hdisplay > 0 && crtc->mode.vdisplay > 0) {
      chosen_crtc = cid;
      chosen_index = (uint32_t)i;
      chosen_w = (uint32_t)crtc->mode.hdisplay;
      chosen_h = (uint32_t)crtc->mode.vdisplay;
      drmModeFreeCrtc(crtc);
      break;
    }
    drmModeFreeCrtc(crtc);
    if (!chosen_crtc) { chosen_crtc = cid; chosen_index = (uint32_t)i; }
  }
  drmModeFreeEncoder(enc);
  drmModeFreeResources(res);
  if (!chosen_crtc || chosen_w == 0 || chosen_h == 0) return -1;
  *out_crtc_id = chosen_crtc;
  *out_crtc_index = chosen_index;
  *out_w = chosen_w;
  *out_h = chosen_h;
  return 0;
}

static bool connector_find_mode(drmModeConnector *conn, uint32_t w, uint32_t h, double vrefresh_hz,
                               drmModeModeInfo *out_mode) {
  if (!conn || conn->count_modes <= 0) return false;
  for (int i = 0; i < conn->count_modes; i++) {
    drmModeModeInfo m = conn->modes[i];
    if ((uint32_t)m.hdisplay != w || (uint32_t)m.vdisplay != h) continue;
    double refresh = 0.0;
    if (m.htotal > 0 && m.vtotal > 0)
      refresh = (double)m.clock * 1000.0 / ((double)m.htotal * (double)m.vtotal);
    if (vrefresh_hz > 0.0) {
      if (refresh > (vrefresh_hz - 1.0) && refresh < (vrefresh_hz + 1.0)) { *out_mode = m; return true; }
    } else {
      *out_mode = m; return true;
    }
  }
  return false;
}

static int drm_modeset_match_input(int drm_fd, uint32_t conn_id, drmModeConnector *conn, uint32_t crtc_id,
                                   uint32_t want_w, uint32_t want_h) {
  drmModeModeInfo chosen;
  memset(&chosen, 0, sizeof(chosen));
  if (!connector_find_mode(conn, want_w, want_h, 60.0, &chosen)) {
    if (!connector_find_mode(conn, want_w, want_h, -1.0, &chosen)) {
      fprintf(stderr, "[modeset] No connector mode for %ux%u\n", want_w, want_h);
      return -1;
    }
  }
  uint32_t mode_blob = 0;
  if (drmModeCreatePropertyBlob(drm_fd, &chosen, sizeof(chosen), &mode_blob) != 0) {
    fprintf(stderr, "[modeset] CreatePropertyBlob failed: %s\n", strerror(errno));
    return -1;
  }
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
  if (ret != 0) {
    fprintf(stderr, "[modeset] commit failed: %s\n", strerror(errno));
    return -1;
  }
  fprintf(stderr, "[modeset] committed.\n");
  return 0;
}

// ---------------- Crosshair + Present ----------------
enum crosshair_mode { XH_INVERT = 0, XH_SOLID = 1 };

struct crosshair_cfg {
  bool enabled = false;
  uint32_t diam_w = 0;
  uint32_t diam_h = 0;
  bool center_set = false;
  int32_t cx = 0;
  int32_t cy = 0;
  uint32_t thickness = 1;
  enum crosshair_mode mode = XH_INVERT;
  uint8_t solid_r = 255, solid_g = 255, solid_b = 255;

  int32_t out_cx = 0, out_cy = 0;
  int32_t half_w = 0, half_h = 0, half_th = 0;
};

struct present_rect {
  int32_t crtc_x, crtc_y;
  uint32_t crtc_w, crtc_h;
  uint32_t src_w, src_h;
};

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
  int rr = -1, gg = -1, bb = -1;
  if (sscanf(s, "%d,%d,%d", &rr, &gg, &bb) != 3) return false;
  if (rr < 0 || rr > 255 || gg < 0 || gg > 255 || bb < 0 || bb > 255) return false;
  *r = (uint8_t)rr; *g = (uint8_t)gg; *b = (uint8_t)bb;
  return true;
}
static bool parse_point_csv(const char *s, int32_t *x, int32_t *y) {
  if (!s) return false;
  int xx = 0, yy = 0;
  if (sscanf(s, "%d,%d", &xx, &yy) != 2) return false;
  *x = (int32_t)xx;
  *y = (int32_t)yy;
  return true;
}

static inline bool out_pixel_is_on_crosshair(const struct crosshair_cfg *xh, int32_t ox, int32_t oy) {
  if (oy >= (xh->out_cy - xh->half_th) && oy <= (xh->out_cy + xh->half_th) &&
      ox >= (xh->out_cx - xh->half_w)  && ox <= (xh->out_cx + xh->half_w)) return true;
  if (ox >= (xh->out_cx - xh->half_th) && ox <= (xh->out_cx + xh->half_th) &&
      oy >= (xh->out_cy - xh->half_h)  && oy <= (xh->out_cy + xh->half_h)) return true;
  return false;
}

static inline void src_to_out_nearest(const struct present_rect *pr, uint32_t sx, uint32_t sy, int32_t *ox, int32_t *oy) {
  *ox = pr->crtc_x + (int32_t)((uint64_t)sx * (uint64_t)pr->crtc_w / (uint64_t)pr->src_w);
  *oy = pr->crtc_y + (int32_t)((uint64_t)sy * (uint64_t)pr->crtc_h / (uint64_t)pr->src_h);
}

static inline void out_to_src_nearest(const struct present_rect *pr, int32_t ox, int32_t oy, uint32_t *sx, uint32_t *sy) {
  int32_t rx = ox - pr->crtc_x;
  int32_t ry = oy - pr->crtc_y;
  if (rx < 0) rx = 0;
  if (ry < 0) ry = 0;
  if (pr->crtc_w == 0 || pr->crtc_h == 0 || pr->src_w == 0 || pr->src_h == 0) { *sx = 0; *sy = 0; return; }

  uint64_t xs = (uint64_t)rx * (uint64_t)pr->src_w / (uint64_t)pr->crtc_w;
  uint64_t ys = (uint64_t)ry * (uint64_t)pr->src_h / (uint64_t)pr->crtc_h;
  if (xs >= pr->src_w) xs = pr->src_w - 1;
  if (ys >= pr->src_h) ys = pr->src_h - 1;
  *sx = (uint32_t)xs;
  *sy = (uint32_t)ys;
}

static inline uint32_t src_read_xrgb8888(const uint8_t *src_rgb, uint32_t src_stride,
                                         uint32_t sx, uint32_t sy, bool input_is_bgr) {
  const uint8_t *p = src_rgb + (uint64_t)sy * src_stride + (uint64_t)sx * 3ull;
  uint8_t r,g,b;
  if (!input_is_bgr) { r = p[0]; g = p[1]; b = p[2]; }
  else               { b = p[0]; g = p[1]; r = p[2]; }
  return 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static inline uint32_t invert_xrgb8888(uint32_t xrgb) {
  return (xrgb & 0xFF000000u) | ((xrgb ^ 0x00FFFFFFu) & 0x00FFFFFFu);
}

static void crosshair_finalize_for_output(crosshair_cfg &xh, uint32_t out_w, uint32_t out_h) {
  if (!xh.enabled) return;
  if (xh.diam_w == 0 || xh.diam_h == 0) { xh.diam_w = 200; xh.diam_h = 200; }
  xh.out_cx = xh.center_set ? xh.cx : (int32_t)(out_w / 2);
  xh.out_cy = xh.center_set ? xh.cy : (int32_t)(out_h / 2);
  xh.half_w = (int32_t)xh.diam_w / 2;
  xh.half_h = (int32_t)xh.diam_h / 2;
  xh.half_th = (int32_t)(xh.thickness ? xh.thickness : 1) / 2;
}

static bool present_policy_valid(const std::string &p) {
  return p == "stretch" || p == "fit" || p == "1:1";
}

static struct present_rect compute_present_rect(const char *policy,
                                                uint32_t out_w, uint32_t out_h,
                                                uint32_t in_w, uint32_t in_h) {
  struct present_rect r;
  memset(&r, 0, sizeof(r));
  r.src_w = in_w;
  r.src_h = in_h;

  if (strcmp(policy, "stretch") == 0) {
    r.crtc_x = 0; r.crtc_y = 0;
    r.crtc_w = out_w; r.crtc_h = out_h;
    return r;
  }
  if (strcmp(policy, "1:1") == 0) {
    r.crtc_w = in_w;
    r.crtc_h = in_h;
    r.crtc_x = (int32_t)((int)out_w - (int)r.crtc_w) / 2;
    r.crtc_y = (int32_t)((int)out_h - (int)r.crtc_h) / 2;
    return r;
  }
  double sx = (in_w > 0) ? ((double)out_w / (double)in_w) : 1.0;
  double sy = (in_h > 0) ? ((double)out_h / (double)in_h) : 1.0;
  double s = (sx < sy) ? sx : sy;
  if (s <= 0.0) s = 1.0;
  uint32_t w = (uint32_t)((double)in_w * s + 0.5);
  uint32_t h = (uint32_t)((double)in_h * s + 0.5);
  if (w > out_w) w = out_w;
  if (h > out_h) h = out_h;
  r.crtc_w = w;
  r.crtc_h = h;
  r.crtc_x = (int32_t)((int)out_w - (int)w) / 2;
  r.crtc_y = (int32_t)((int)out_h - (int)h) / 2;
  return r;
}

static int drm_commit_plane_present(
  int drm_fd,
  uint32_t plane_id, uint32_t crtc_id,
  uint32_t prop_fb_id, uint32_t prop_crtc_id,
  uint32_t prop_crtc_x, uint32_t prop_crtc_y, uint32_t prop_crtc_w, uint32_t prop_crtc_h,
  uint32_t prop_src_x, uint32_t prop_src_y, uint32_t prop_src_w, uint32_t prop_src_h,
  uint32_t fb_id_to_use,
  const struct present_rect *pr
) {
  drmModeAtomicReq *req = drmModeAtomicAlloc();
  if (!req) die("drmModeAtomicAlloc");
  drmModeAtomicAddProperty(req, plane_id, prop_fb_id, fb_id_to_use);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_id, crtc_id);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_x, (uint32_t)pr->crtc_x);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_y, (uint32_t)pr->crtc_y);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_w, pr->crtc_w);
  drmModeAtomicAddProperty(req, plane_id, prop_crtc_h, pr->crtc_h);
  drmModeAtomicAddProperty(req, plane_id, prop_src_x, 0 << 16);
  drmModeAtomicAddProperty(req, plane_id, prop_src_y, 0 << 16);
  drmModeAtomicAddProperty(req, plane_id, prop_src_w, pr->src_w << 16);
  drmModeAtomicAddProperty(req, plane_id, prop_src_h, pr->src_h << 16);
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

// ---------------- Conversion ----------------
static inline void rgb24_to_xrgb8888_scalar_rows_xh(
  uint8_t *dst_xrgb, uint32_t dst_stride,
  const uint8_t *src_rgb, uint32_t src_stride,
  uint32_t w, uint32_t y0, uint32_t y1,
  bool input_is_bgr,
  const struct present_rect *pr,
  const struct crosshair_cfg *xh
) {
  for (uint32_t y = y0; y < y1; y++) {
    uint32_t *d = (uint32_t *)(dst_xrgb + y * dst_stride);
    const uint8_t *s = src_rgb + (uint64_t)y * src_stride;
    for (uint32_t x = 0; x < w; x++) {
      uint8_t r, g, b;
      const uint8_t *p = s + (uint64_t)x * 3ull;
      if (!input_is_bgr) { r = p[0]; g = p[1]; b = p[2]; }
      else               { b = p[0]; g = p[1]; r = p[2]; }
      uint32_t px = 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;

      if (xh && xh->enabled) {
        int32_t ox, oy;
        src_to_out_nearest(pr, x, y, &ox, &oy);
        if (out_pixel_is_on_crosshair(xh, ox, oy)) {
          if (xh->mode == XH_SOLID) {
            px = 0xFF000000u | ((uint32_t)xh->solid_r << 16) | ((uint32_t)xh->solid_g << 8) | (uint32_t)xh->solid_b;
          } else {
            uint32_t sx, sy;
            out_to_src_nearest(pr, ox, oy, &sx, &sy);
            uint32_t under = src_read_xrgb8888(src_rgb, src_stride, sx, sy, input_is_bgr);
            px = invert_xrgb8888(under);
          }
        }
      }

      d[x] = px;
    }
  }
}

#if HAVE_NEON
static inline void rgb24_to_xrgb8888_neon_rows_xh(
  uint8_t *dst_xrgb, uint32_t dst_stride,
  const uint8_t *src_rgb, uint32_t src_stride,
  uint32_t w, uint32_t y0, uint32_t y1,
  bool input_is_bgr,
  const struct present_rect *pr,
  const struct crosshair_cfg *xh
) {
  for (uint32_t y = y0; y < y1; y++) {
    uint32_t *d = (uint32_t *)(dst_xrgb + y * dst_stride);
    const uint8_t *s = src_rgb + (uint64_t)y * src_stride;
    uint32_t x = 0;

    for (; x + 16 <= w; x += 16) {
      uint8x16x3_t v = vld3q_u8(s + (uint64_t)x * 3ull);
      uint8x16_t r = input_is_bgr ? v.val[2] : v.val[0];
      uint8x16_t g = v.val[1];
      uint8x16_t b = input_is_bgr ? v.val[0] : v.val[2];

      uint16x8_t r0 = vmovl_u8(vget_low_u8(r));
      uint16x8_t g0 = vmovl_u8(vget_low_u8(g));
      uint16x8_t b0 = vmovl_u8(vget_low_u8(b));
      uint16x8_t r1 = vmovl_u8(vget_high_u8(r));
      uint16x8_t g1 = vmovl_u8(vget_high_u8(g));
      uint16x8_t b1 = vmovl_u8(vget_high_u8(b));

      uint32x4_t p0 = vorrq_u32(vdupq_n_u32(0xFF000000u),
                               vorrq_u32(vshlq_n_u32(vmovl_u16(vget_low_u16(r0)), 16),
                                         vorrq_u32(vshlq_n_u32(vmovl_u16(vget_low_u16(g0)), 8),
                                                   vmovl_u16(vget_low_u16(b0)))));
      uint32x4_t p1 = vorrq_u32(vdupq_n_u32(0xFF000000u),
                               vorrq_u32(vshlq_n_u32(vmovl_u16(vget_high_u16(r0)), 16),
                                         vorrq_u32(vshlq_n_u32(vmovl_u16(vget_high_u16(g0)), 8),
                                                   vmovl_u16(vget_high_u16(b0)))));
      uint32x4_t p2 = vorrq_u32(vdupq_n_u32(0xFF000000u),
                               vorrq_u32(vshlq_n_u32(vmovl_u16(vget_low_u16(r1)), 16),
                                         vorrq_u32(vshlq_n_u32(vmovl_u16(vget_low_u16(g1)), 8),
                                                   vmovl_u16(vget_low_u16(b1)))));
      uint32x4_t p3 = vorrq_u32(vdupq_n_u32(0xFF000000u),
                               vorrq_u32(vshlq_n_u32(vmovl_u16(vget_high_u16(r1)), 16),
                                         vorrq_u32(vshlq_n_u32(vmovl_u16(vget_high_u16(g1)), 8),
                                                   vmovl_u16(vget_high_u16(b1)))));

      vst1q_u32(d + x + 0,  p0);
      vst1q_u32(d + x + 4,  p1);
      vst1q_u32(d + x + 8,  p2);
      vst1q_u32(d + x + 12, p3);

      if (xh && xh->enabled) {
        for (uint32_t i = 0; i < 16; i++) {
          int32_t ox, oy;
          src_to_out_nearest(pr, x + i, y, &ox, &oy);
          if (out_pixel_is_on_crosshair(xh, ox, oy)) {
            if (xh->mode == XH_SOLID) {
              d[x + i] = 0xFF000000u | ((uint32_t)xh->solid_r << 16) | ((uint32_t)xh->solid_g << 8) | (uint32_t)xh->solid_b;
            } else {
              uint32_t sx, sy;
              out_to_src_nearest(pr, ox, oy, &sx, &sy);
              uint32_t under = src_read_xrgb8888(src_rgb, src_stride, sx, sy, input_is_bgr);
              d[x + i] = invert_xrgb8888(under);
            }
          }
        }
      }
    }

    for (; x < w; x++) {
      uint8_t r,g,b;
      const uint8_t *p = s + (uint64_t)x * 3ull;
      if (!input_is_bgr) { r = p[0]; g = p[1]; b = p[2]; }
      else               { b = p[0]; g = p[1]; r = p[2]; }
      uint32_t px = 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;

      if (xh && xh->enabled) {
        int32_t ox, oy;
        src_to_out_nearest(pr, x, y, &ox, &oy);
        if (out_pixel_is_on_crosshair(xh, ox, oy)) {
          if (xh->mode == XH_SOLID) {
            px = 0xFF000000u | ((uint32_t)xh->solid_r << 16) | ((uint32_t)xh->solid_g << 8) | (uint32_t)xh->solid_b;
          } else {
            uint32_t sx, sy;
            out_to_src_nearest(pr, ox, oy, &sx, &sy);
            uint32_t under = src_read_xrgb8888(src_rgb, src_stride, sx, sy, input_is_bgr);
            px = invert_xrgb8888(under);
          }
        }
      }
      d[x] = px;
    }
  }
}
#endif

// ---------------- Thread pool ----------------
struct conv_job {
  uint8_t *dst;
  uint32_t dst_stride;
  const uint8_t *src;
  uint32_t src_stride;
  uint32_t w, h;
  bool input_is_bgr;
  bool use_neon;
  struct present_rect pr;
  struct crosshair_cfg xh;
};

struct worker {
  pthread_t th;
  int id;
  int cpu;
  uint32_t y0, y1;
};

static pthread_mutex_t g_pool_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_pool_cv_job = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  g_pool_cv_done = PTHREAD_COND_INITIALIZER;
static struct conv_job g_pool_job;
static bool g_pool_have_job = false;
static bool g_pool_stop = false;
static int g_pool_workers = 0;
static int g_pool_done = 0;
static struct worker *g_pool_ws = NULL;

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

static void *worker_main(void *arg) {
  struct worker *w = (struct worker*)arg;
  if (w->cpu >= 0) pin_thread_to_cpu(w->cpu);
  for (;;) {
    pthread_mutex_lock(&g_pool_mtx);
    while (!g_pool_have_job && !g_pool_stop) pthread_cond_wait(&g_pool_cv_job, &g_pool_mtx);
    if (g_pool_stop) { pthread_mutex_unlock(&g_pool_mtx); return NULL; }
    struct conv_job job = g_pool_job;
    uint32_t y0 = w->y0, y1 = w->y1;
    pthread_mutex_unlock(&g_pool_mtx);

#if HAVE_NEON
    if (job.use_neon) {
      rgb24_to_xrgb8888_neon_rows_xh(job.dst, job.dst_stride, job.src, job.src_stride,
                                     job.w, y0, y1, job.input_is_bgr,
                                     &job.pr, job.xh.enabled ? &job.xh : NULL);
    } else
#endif
    {
      rgb24_to_xrgb8888_scalar_rows_xh(job.dst, job.dst_stride, job.src, job.src_stride,
                                       job.w, y0, y1, job.input_is_bgr,
                                       &job.pr, job.xh.enabled ? &job.xh : NULL);
    }

    pthread_mutex_lock(&g_pool_mtx);
    g_pool_done++;
    if (g_pool_done == g_pool_workers) {
      g_pool_have_job = false;
      pthread_cond_signal(&g_pool_cv_done);
    }
    pthread_mutex_unlock(&g_pool_mtx);
  }
}

static int conv_pool_init(int threads, uint32_t h) {
  g_pool_workers = threads;
  g_pool_stop = false;
  g_pool_ws = (struct worker*)calloc((size_t)g_pool_workers, sizeof(*g_pool_ws));
  if (!g_pool_ws) return -1;
  int cpu_count = (int)sysconf(_SC_NPROCESSORS_ONLN);
  if (cpu_count < 1) cpu_count = 1;
  for (int i = 0; i < g_pool_workers; i++) {
    g_pool_ws[i].id = i;
    g_pool_ws[i].cpu = (i < cpu_count) ? i : (i % cpu_count);
    uint32_t y0 = (uint32_t)((uint64_t)h * (uint64_t)i / (uint64_t)g_pool_workers);
    uint32_t y1 = (uint32_t)((uint64_t)h * (uint64_t)(i + 1) / (uint64_t)g_pool_workers);
    g_pool_ws[i].y0 = y0;
    g_pool_ws[i].y1 = y1;
    if (pthread_create(&g_pool_ws[i].th, NULL, worker_main, &g_pool_ws[i]) != 0) {
      fprintf(stderr, "[conv] pthread_create failed\n");
      return -1;
    }
  }
  return 0;
}

static void conv_pool_destroy(void) {
  pthread_mutex_lock(&g_pool_mtx);
  g_pool_stop = true;
  pthread_cond_broadcast(&g_pool_cv_job);
  pthread_mutex_unlock(&g_pool_mtx);
  for (int i = 0; i < g_pool_workers; i++) pthread_join(g_pool_ws[i].th, NULL);
  free(g_pool_ws);
  g_pool_ws = NULL;
  g_pool_workers = 0;
}

static void conv_pool_convert_frame(const struct conv_job *job) {
  pthread_mutex_lock(&g_pool_mtx);
  g_pool_job = *job;
  g_pool_done = 0;
  g_pool_have_job = true;
  pthread_cond_broadcast(&g_pool_cv_job);
  while (g_pool_have_job) pthread_cond_wait(&g_pool_cv_done, &g_pool_mtx);
  pthread_mutex_unlock(&g_pool_mtx);
}

// ---------------- Config + JSON (minimal) ----------------
static std::string json_escape(const std::string &s) {
  std::string o;
  o.reserve(s.size() + 8);
  for (char c : s) {
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
  if (v.rfind("true", 0) == 0) { out = true; return true; }
  if (v.rfind("false", 0) == 0) { out = false; return true; }
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
  out = atoi(v.substr(first, i - first).c_str());
  return true;
}

struct persisted_config {
  std::string v4l2_dev = "/dev/video0";
  std::string edid_path;
  bool do_modeset = false;
  int threads = 0;
  bool input_is_bgr = false;
  std::string present_policy = "stretch";
  crosshair_cfg xh{};
};

static std::string config_to_json(const persisted_config &c) {
  std::ostringstream ss;
  ss << "{";
  ss << "\"v4l2Dev\":\"" << json_escape(c.v4l2_dev) << "\",";
  ss << "\"edidPath\":\"" << json_escape(c.edid_path) << "\",";
  ss << "\"modesetMatchInput\":" << (c.do_modeset ? "true" : "false") << ",";
  ss << "\"threads\":" << c.threads << ",";
  ss << "\"bgr\":" << (c.input_is_bgr ? "true" : "false") << ",";
  ss << "\"present\":\"" << json_escape(c.present_policy) << "\",";
  ss << "\"crosshairEnabled\":" << (c.xh.enabled ? "true" : "false") << ",";
  ss << "\"crosshairDiam\":\"" << c.xh.diam_w << "x" << c.xh.diam_h << "\",";
  ss << "\"crosshairCenter\":\"" << (c.xh.center_set ? (std::to_string(c.xh.cx) + "," + std::to_string(c.xh.cy)) : "") << "\",";
  ss << "\"crosshairMode\":\"" << (c.xh.mode == XH_SOLID ? "solid" : "invert") << "\",";
  ss << "\"crosshairColor\":\"" << (int)c.xh.solid_r << "," << (int)c.xh.solid_g << "," << (int)c.xh.solid_b << "\",";
  ss << "\"crosshairThickness\":" << c.xh.thickness;
  ss << "}";
  return ss.str();
}

static bool config_from_json(const std::string &j, persisted_config &c) {
  std::string s;
  bool b;
  int n;

  if (json_get_string(j, "v4l2Dev", s)) c.v4l2_dev = s;
  if (json_get_string(j, "edidPath", s)) c.edid_path = s;
  if (json_get_bool(j, "modesetMatchInput", b)) c.do_modeset = b;
  if (json_get_int(j, "threads", n)) c.threads = n;
  if (json_get_bool(j, "bgr", b)) c.input_is_bgr = b;
  if (json_get_string(j, "present", s) && present_policy_valid(s)) c.present_policy = s;

  if (json_get_bool(j, "crosshairEnabled", b)) c.xh.enabled = b;
  if (json_get_string(j, "crosshairDiam", s)) {
    uint32_t w=0,h=0;
    if (s.size() && parse_dim(s.c_str(), &w, &h)) { c.xh.diam_w = w; c.xh.diam_h = h; }
  }
  if (json_get_string(j, "crosshairCenter", s)) {
    if (s.empty()) c.xh.center_set = false;
    else {
      int32_t cx=0, cy=0;
      if (parse_point_csv(s.c_str(), &cx, &cy)) { c.xh.center_set = true; c.xh.cx=cx; c.xh.cy=cy; }
    }
  }
  if (json_get_string(j, "crosshairMode", s)) {
    if (s == "invert") c.xh.mode = XH_INVERT;
    else if (s == "solid") c.xh.mode = XH_SOLID;
  }
  if (json_get_string(j, "crosshairColor", s)) {
    uint8_t r=0,g=0,b2=0;
    if (parse_rgb_csv(s.c_str(), &r, &g, &b2)) { c.xh.solid_r=r; c.xh.solid_g=g; c.xh.solid_b=b2; }
  }
  if (json_get_int(j, "crosshairThickness", n)) {
    if (n >= 1 && n <= 99) c.xh.thickness = (uint32_t)n;
  }
  return true;
}

// ---------------- Global state ----------------
static std::mutex g_cfg_mtx;
static persisted_config g_cfg;
static std::string g_cfg_path = "./overlay_config.json";

static std::atomic<bool> g_quit{false};
static std::atomic<bool> g_reinit_requested{false};
static std::mutex g_reinit_mtx;
static std::string g_reinit_reason;

static void request_reinit(const std::string &reason) {
  std::lock_guard<std::mutex> lk(g_reinit_mtx);
  g_reinit_reason = reason;
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

// ---------------- Pipeline ----------------
struct pipeline {
  int vfd = -1;
  cap_buf *cbufs = NULL;
  uint32_t cbuf_count = 0;
  uint32_t in_w = 0, in_h = 0, in_stride = 0;

  int drm_fd = -1;
  drmModeConnector *conn = NULL;
  uint32_t conn_id = 0;
  uint32_t crtc_id = 0, crtc_index = 0, out_w = 0, out_h = 0;
  uint32_t plane_id = 0;

  uint32_t p_fb_id=0, p_crtc_id=0, p_crtc_x=0, p_crtc_y=0, p_crtc_w=0, p_crtc_h=0;
  uint32_t p_src_x=0, p_src_y=0, p_src_w=0, p_src_h=0;

  dumb_fb fb[2];
  int cur = 0;
  present_rect pr{};
  bool dumped = false;
  bool saw_frame = false;

  bool init_from_config(const persisted_config &cfg) {
    memset(&fb[0], 0, sizeof(fb[0]));
    memset(&fb[1], 0, sizeof(fb[1]));

    int cpu_count = (int)sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count < 1) cpu_count = 1;
    int threads = cfg.threads;
    if (threads <= 0) threads = cpu_count;
    if (threads < 1) threads = 1;
    if (threads > cpu_count) threads = cpu_count;

    fprintf(stderr, "[pipeline] init v4l2=%s threads=%d bgr=%d present=%s\n",
            cfg.v4l2_dev.c_str(), threads, cfg.input_is_bgr ? 1 : 0, cfg.present_policy.c_str());

    vfd = v4l2_open(cfg.v4l2_dev.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (vfd < 0) { perror("v4l2_open"); return false; }

    if (!cfg.edid_path.empty()) {
      (void)v4l2_set_edid_if_requested(vfd, cfg.edid_path.c_str());
      usleep(2000 * 1000);
    }

    uint32_t timing_w = 0, timing_h = 0;
    uint64_t pixelclock = 0;
    if (tc358743_query_and_set_dv_timings(vfd, &timing_w, &timing_h, &pixelclock) != 0) {
      fprintf(stderr, "[pipeline] No DV timings.\n");
      return false;
    }

    if (tc358743_set_pixfmt_rgb24(vfd, &in_w, &in_h, &in_stride) != 0) return false;
    if (v4l2_start_mmap_capture(vfd, &cbufs, &cbuf_count) != 0) return false;

    if (conv_pool_init(threads, in_h) != 0) {
      fprintf(stderr, "[conv] failed to init thread pool\n");
      return false;
    }

    drm_fd = open_vc4_card();
    if (drm_fd < 0) { perror("open /dev/dri/card"); return false; }
    if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) { perror("drmSetClientCap"); return false; }

    if (find_hdmi_a_1(drm_fd, &conn_id, &conn) != 0) { fprintf(stderr, "[drm] find_hdmi_a_1 failed\n"); return false; }
    if (conn->connection != DRM_MODE_CONNECTED) { fprintf(stderr, "[drm] HDMI-A-1 not connected\n"); return false; }

    if (get_active_crtc_for_connector(drm_fd, conn, &crtc_id, &crtc_index, &out_w, &out_h) != 0) {
      fprintf(stderr, "[drm] No active CRTC mode yet.\n");
      return false;
    }

    if (cfg.do_modeset) {
      fprintf(stderr, "[pipeline] modeset match input: %ux%u\n", in_w, in_h);
      (void)drm_modeset_match_input(drm_fd, conn_id, conn, crtc_id, in_w, in_h);
      (void)get_active_crtc_for_connector(drm_fd, conn, &crtc_id, &crtc_index, &out_w, &out_h);
    }

    const uint32_t fmt = DRM_FORMAT_XRGB8888;
    plane_id = find_primary_plane_on_crtc(drm_fd, crtc_index, fmt);
    if (!plane_id) {
      fprintf(stderr, "[drm] Could not find PRIMARY plane supporting XRGB8888\n");
      return false;
    }

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
        !p_src_x || !p_src_y || !p_src_w || !p_src_h) {
      fprintf(stderr, "[drm] Missing plane properties\n");
      return false;
    }

    pr = compute_present_rect(cfg.present_policy.c_str(), out_w, out_h, in_w, in_h);
    fprintf(stderr, "[present] policy=%s -> crtc=%ux%u at (%d,%d)\n",
            cfg.present_policy.c_str(), pr.crtc_w, pr.crtc_h, pr.crtc_x, pr.crtc_y);

    if (dumb_fb_create(drm_fd, in_w, in_h, fmt, &fb[0]) != 0) return false;
    if (dumb_fb_create(drm_fd, in_w, in_h, fmt, &fb[1]) != 0) return false;

    {
      uint32_t *d = (uint32_t*)fb[0].map;
      uint32_t stride_u32 = fb[0].pitch / 4;
      for (uint32_t y = 0; y < in_h; y++) {
        for (uint32_t x = 0; x < in_w; x++) {
          uint32_t r = (x * 255) / (in_w ? in_w : 1);
          uint32_t g = (y * 255) / (in_h ? in_h : 1);
          uint32_t bb = 0x40;
          d[y * stride_u32 + x] = 0xFF000000u | (r << 16) | (g << 8) | bb;
        }
      }
    }

    int ret = drm_commit_plane_present(drm_fd, plane_id, crtc_id,
                                       p_fb_id, p_crtc_id, p_crtc_x, p_crtc_y, p_crtc_w, p_crtc_h,
                                       p_src_x, p_src_y, p_src_w, p_src_h,
                                       fb[0].fb_id, &pr);
    if (ret != 0) {
      fprintf(stderr, "[drm] Initial commit failed: %s\n", strerror(errno));
      return false;
    }

    cur = 0;
    dumped = false;
    saw_frame = false;
    return true;
  }

  void shutdown() {
    if (drm_fd >= 0 && plane_id) {
      drmModeAtomicReq *off = drmModeAtomicAlloc();
      if (off) {
        drmModeAtomicAddProperty(off, plane_id, p_fb_id, 0);
        drmModeAtomicAddProperty(off, plane_id, p_crtc_id, 0);
        drmModeAtomicCommit(drm_fd, off, 0, NULL);
        drmModeAtomicFree(off);
      }
    }

    if (drm_fd >= 0) {
      dumb_fb_destroy(drm_fd, &fb[0]);
      dumb_fb_destroy(drm_fd, &fb[1]);
    }

    if (g_pool_workers > 0) conv_pool_destroy();

    if (vfd >= 0) {
      v4l2_stop_and_unmap(vfd, cbufs, cbuf_count);
      cbufs = NULL; cbuf_count = 0;
      v4l2_close(vfd);
      vfd = -1;
    }

    if (conn) { drmModeFreeConnector(conn); conn = NULL; }
    if (drm_fd >= 0) { close(drm_fd); drm_fd = -1; }
  }

  bool step_frame(const persisted_config &cfg_live) {
    present_rect npr = compute_present_rect(cfg_live.present_policy.c_str(), out_w, out_h, in_w, in_h);
    if (memcmp(&npr, &pr, sizeof(pr)) != 0) {
      int rr = drm_commit_plane_present(drm_fd, plane_id, crtc_id,
                                        p_fb_id, p_crtc_id, p_crtc_x, p_crtc_y, p_crtc_w, p_crtc_h,
                                        p_src_x, p_src_y, p_src_w, p_src_h,
                                        fb[cur].fb_id, &npr);
      if (rr == 0) pr = npr;
    }

    int prc = v4l2_wait_for_frame(vfd, 100);
    if (prc < 0) return false;
    if (prc == 0) return true;

    struct v4l2_buffer b;
    int dq = v4l2_dequeue_latest_frame(vfd, &b);
    if (dq == 1) return true;
    if (dq < 0) return false;

    const uint8_t *src = (const uint8_t *)cbufs[b.index].start;
    if (!dumped) { dump_first_pixels_rgb24(src); dumped = true; }

    int next = cur ^ 1;

    crosshair_cfg xh = cfg_live.xh;
    crosshair_finalize_for_output(xh, out_w, out_h);

    struct conv_job job;
    job.dst = (uint8_t*)fb[next].map;
    job.dst_stride = fb[next].pitch;
    job.src = src;
    job.src_stride = in_stride;
    job.w = in_w;
    job.h = in_h;
    job.input_is_bgr = cfg_live.input_is_bgr;
    job.use_neon = true;
    job.pr = pr;
    job.xh = xh;

    conv_pool_convert_frame(&job);

    if (v4l2_queue_frame(vfd, &b) != 0) return false;

    if (!saw_frame) {
      fprintf(stderr, "[v4l2] First frame received. Switching from test pattern to live video.\n");
      saw_frame = true;
    }

    int rr = drm_plane_flip_fb_only(drm_fd, plane_id, p_fb_id, fb[next].fb_id);
    if (rr != 0) {
      fprintf(stderr, "[drm] FB flip commit failed: %s\n", strerror(errno));
      return false;
    }

    cur = next;
    return true;
  }
};

// ---------------- WebUI ----------------
static const char *kIndexHtml = R"HTML(<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>DRM Overlay Controller</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 18px; max-width: 900px; }
    fieldset { margin: 12px 0; padding: 12px; }
    label { display: block; margin: 8px 0 4px; font-weight: 600; }
    input[type="text"], input[type="number"], select { width: 100%; padding: 8px; box-sizing: border-box; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .buttons { display: flex; gap: 10px; margin-top: 12px; flex-wrap: wrap; }
    button { padding: 10px 14px; cursor: pointer; }
    .status { padding: 10px; background: #f5f5f5; border: 1px solid #ddd; white-space: pre-wrap; }
    .muted { color: #666; font-size: 0.9em; }
    code { background: #eee; padding: 1px 4px; border-radius: 3px; }
  </style>
</head>
<body>
  <h1>DRM Overlay Controller</h1>
  <div class="status" id="status">Loading...</div>

  <fieldset>
    <legend>Video / Present</legend>
    <div class="row">
      <div>
        <label>V4L2 Device</label>
        <input id="v4l2Dev" type="text" placeholder="/dev/video0" />
      </div>
      <div>
        <label>EDID Path</label>
        <input id="edidPath" type="text" placeholder="/root/drm_overlay/720P60EDID.txt" />
      </div>
    </div>

    <div class="row">
      <div>
        <label>Present Policy</label>
        <select id="present">
          <option value="stretch">stretch</option>
          <option value="fit">fit</option>
          <option value="1:1">1:1</option>
        </select>
      </div>
      <div>
        <label>Threads</label>
        <input id="threads" type="number" min="1" max="64" step="1" />
      </div>
    </div>

    <div class="row">
      <div>
        <label><input id="bgr" type="checkbox" /> Input is BGR</label>
      </div>
      <div>
        <label><input id="modesetMatchInput" type="checkbox" /> Modeset HDMI to match input</label>
      </div>
    </div>
  </fieldset>

  <fieldset>
    <legend>Crosshair</legend>
    <label><input id="crosshairEnabled" type="checkbox" /> Enable crosshair</label>

    <div class="row">
      <div>
        <label>Diameter (DIAM or WxH)</label>
        <input id="crosshairDiam" type="text" placeholder="50x50 or 200" />
      </div>
      <div>
        <label>Center (X,Y) optional</label>
        <input id="crosshairCenter" type="text" placeholder="e.g. 640,360" />
      </div>
    </div>

    <div class="row">
      <div>
        <label>Mode</label>
        <select id="crosshairMode">
          <option value="invert">invert</option>
          <option value="solid">solid</option>
        </select>
      </div>
      <div>
        <label>Thickness</label>
        <input id="crosshairThickness" type="number" min="1" max="99" step="1" />
      </div>
    </div>

    <label>Solid Color (R,G,B)</label>
    <input id="crosshairColor" type="text" placeholder="255,255,255" />
  </fieldset>

  <div class="buttons">
    <button id="apply">Apply</button>
    <button id="quit">Quit Program</button>
  </div>

  <script>
    async function api(path, opts) {
      const r = await fetch(path, opts);
      const t = await r.text();
      let j;
      try { j = JSON.parse(t); } catch { j = { raw: t }; }
      if (!r.ok) throw new Error(j.error || t);
      return j;
    }

    function getForm() {
      return {
        v4l2Dev: document.getElementById('v4l2Dev').value,
        edidPath: document.getElementById('edidPath').value,
        bgr: document.getElementById('bgr').checked,
        modesetMatchInput: document.getElementById('modesetMatchInput').checked,
        present: document.getElementById('present').value,
        threads: Number(document.getElementById('threads').value),

        crosshairEnabled: document.getElementById('crosshairEnabled').checked,
        crosshairDiam: document.getElementById('crosshairDiam').value,
        crosshairCenter: document.getElementById('crosshairCenter').value,
        crosshairMode: document.getElementById('crosshairMode').value,
        crosshairColor: document.getElementById('crosshairColor').value,
        crosshairThickness: Number(document.getElementById('crosshairThickness').value),
      };
    }

    function setForm(cfg) {
      document.getElementById('v4l2Dev').value = cfg.v4l2Dev || '/dev/video0';
      document.getElementById('edidPath').value = cfg.edidPath || '';
      document.getElementById('bgr').checked = !!cfg.bgr;
      document.getElementById('modesetMatchInput').checked = !!cfg.modesetMatchInput;
      document.getElementById('present').value = cfg.present || 'stretch';
      document.getElementById('threads').value = cfg.threads || 0;

      document.getElementById('crosshairEnabled').checked = !!cfg.crosshairEnabled;
      document.getElementById('crosshairDiam').value = cfg.crosshairDiam || '';
      document.getElementById('crosshairCenter').value = cfg.crosshairCenter || '';
      document.getElementById('crosshairMode').value = cfg.crosshairMode || 'invert';
      document.getElementById('crosshairColor').value = cfg.crosshairColor || '255,255,255';
      document.getElementById('crosshairThickness').value = cfg.crosshairThickness || 1;
    }

    async function refreshStatus() {
      const st = await api('/api/status');
      document.getElementById('status').textContent =
        `PID: ${st.pid}\n` +
        `reinitRequested: ${st.reinitRequested}\n` +
        `reinitReason: ${st.reinitReason}\n` +
        `configPath: ${st.configPath}\n`;
    }

    async function init() {
      const cfg = await api('/api/config');
      setForm(cfg);
      await refreshStatus();
      setInterval(refreshStatus, 1000);
    }

    document.getElementById('apply').onclick = async () => {
      const cfg = getForm();
      const r = await api('/api/apply', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(cfg) });
      // The backend returns effectiveConfig now; reflect it back into UI
      if (r && r.effectiveConfig) setForm(r.effectiveConfig);
      await refreshStatus();
    };

    document.getElementById('quit').onclick = async () => {
      await api('/api/quit', { method: 'POST' });
    };

    init().catch(e => {
      document.getElementById('status').textContent = 'Error: ' + e.message;
    });
  </script>
</body>
</html>)HTML";

static std::atomic<pid_t> g_pid{0};

static std::string cfg_as_api_json(const persisted_config &c) {
  std::ostringstream ss;
  ss << "{";
  ss << "\"v4l2Dev\":\"" << json_escape(c.v4l2_dev) << "\",";
  ss << "\"edidPath\":\"" << json_escape(c.edid_path) << "\",";
  ss << "\"modesetMatchInput\":" << (c.do_modeset ? "true" : "false") << ",";
  ss << "\"threads\":" << c.threads << ",";
  ss << "\"bgr\":" << (c.input_is_bgr ? "true" : "false") << ",";
  ss << "\"present\":\"" << json_escape(c.present_policy) << "\",";
  ss << "\"crosshairEnabled\":" << (c.xh.enabled ? "true" : "false") << ",";
  ss << "\"crosshairDiam\":\"" << c.xh.diam_w << "x" << c.xh.diam_h << "\",";
  ss << "\"crosshairCenter\":\"" << (c.xh.center_set ? (std::to_string(c.xh.cx) + "," + std::to_string(c.xh.cy)) : "") << "\",";
  ss << "\"crosshairMode\":\"" << (c.xh.mode == XH_SOLID ? "solid" : "invert") << "\",";
  ss << "\"crosshairColor\":\"" << (int)c.xh.solid_r << "," << (int)c.xh.solid_g << "," << (int)c.xh.solid_b << "\",";
  ss << "\"crosshairThickness\":" << c.xh.thickness;
  ss << "}";
  return ss.str();
}

static void webui_thread_main(int port) {
  httplib::Server svr;

  svr.Get("/", [](const httplib::Request&, httplib::Response &res) {
    res.set_content(kIndexHtml, "text/html; charset=utf-8");
  });

  svr.Get("/api/config", [](const httplib::Request&, httplib::Response &res) {
    auto snap = cfg_snapshot();
    res.set_content(cfg_as_api_json(snap), "application/json");
  });

  svr.Get("/api/status", [](const httplib::Request&, httplib::Response &res) {
    std::ostringstream ss;
    ss << "{";
    ss << "\"pid\":" << (int)g_pid.load() << ",";
    ss << "\"reinitRequested\":" << (g_reinit_requested.load() ? "true" : "false") << ",";
    ss << "\"reinitReason\":\"" << json_escape(take_reinit_reason()) << "\",";
    ss << "\"configPath\":\"" << json_escape(g_cfg_path) << "\"";
    ss << "}";
    res.set_content(ss.str(), "application/json");
  });

  svr.Post("/api/apply", [](const httplib::Request &req, httplib::Response &res) {
    auto body = req.body;

    persisted_config cur = cfg_snapshot();
    persisted_config next = cur;

    std::string s;
    bool b;
    int n;

    if (json_get_string(body, "v4l2Dev", s) && !s.empty()) next.v4l2_dev = s;
    if (json_get_string(body, "edidPath", s)) next.edid_path = s;
    if (json_get_bool(body, "modesetMatchInput", b)) next.do_modeset = b;
    if (json_get_int(body, "threads", n)) next.threads = n;
    if (json_get_bool(body, "bgr", b)) next.input_is_bgr = b;
    if (json_get_string(body, "present", s)) {
      if (!present_policy_valid(s)) { res.status=400; res.set_content("{\"error\":\"invalid present\"}", "application/json"); return; }
      next.present_policy = s;
    }

    if (json_get_bool(body, "crosshairEnabled", b)) next.xh.enabled = b;

    if (json_get_string(body, "crosshairDiam", s)) {
      if (!s.empty()) {
        uint32_t w=0,h=0;
        if (!parse_dim(s.c_str(), &w, &h)) { res.status=400; res.set_content("{\"error\":\"invalid crosshairDiam\"}", "application/json"); return; }
        next.xh.diam_w=w; next.xh.diam_h=h;
      }
    }

    if (json_get_string(body, "crosshairCenter", s)) {
      if (s.empty()) next.xh.center_set = false;
      else {
        int32_t cx=0, cy=0;
        if (!parse_point_csv(s.c_str(), &cx, &cy)) { res.status=400; res.set_content("{\"error\":\"invalid crosshairCenter\"}", "application/json"); return; }
        next.xh.center_set = true; next.xh.cx=cx; next.xh.cy=cy;
      }
    }

    // IMPORTANT: persist mode exactly as requested
    if (json_get_string(body, "crosshairMode", s)) {
      if (s == "invert") next.xh.mode = XH_INVERT;
      else if (s == "solid") next.xh.mode = XH_SOLID;
      else { res.status=400; res.set_content("{\"error\":\"invalid crosshairMode\"}", "application/json"); return; }
    }

    // If user provides a color, it implies solid mode (same as before)
    if (json_get_string(body, "crosshairColor", s)) {
      if (!s.empty()) {
        uint8_t rr=0,gg=0,bb=0;
        if (!parse_rgb_csv(s.c_str(), &rr, &gg, &bb)) { res.status=400; res.set_content("{\"error\":\"invalid crosshairColor\"}", "application/json"); return; }
        next.xh.solid_r=rr; next.xh.solid_g=gg; next.xh.solid_b=bb;
        next.xh.mode = XH_SOLID;
      }
    }

    if (json_get_int(body, "crosshairThickness", n)) {
      if (n < 1 || n > 99) { res.status=400; res.set_content("{\"error\":\"invalid crosshairThickness\"}", "application/json"); return; }
      next.xh.thickness = (uint32_t)n;
    }

    bool need_reinit = false;
    std::string reason;
    if (next.v4l2_dev != cur.v4l2_dev) { need_reinit=true; reason="v4l2Dev changed"; }
    else if (next.edid_path != cur.edid_path) { need_reinit=true; reason="edidPath changed"; }
    else if (next.do_modeset != cur.do_modeset) { need_reinit=true; reason="modesetMatchInput changed"; }
    else if (next.threads != cur.threads) { need_reinit=true; reason="threads changed"; }

    // Always persist on apply
    {
      std::lock_guard<std::mutex> lk(g_cfg_mtx);
      g_cfg = next;
      (void)save_config_locked();
    }

    if (need_reinit) request_reinit(reason);

    // Return the effective config so UI can't get out of sync with backend
    std::ostringstream out;
    out << "{";
    out << "\"ok\":true,";
    out << "\"effectiveConfig\":" << cfg_as_api_json(next);
    out << "}";
    res.set_content(out.str(), "application/json");
  });

  svr.Post("/api/quit", [](const httplib::Request&, httplib::Response &res) {
    g_quit.store(true);
    res.set_content("{\"ok\":true}", "application/json");
  });

  fprintf(stderr, "[webui] listening on 0.0.0.0:%d\n", port);
  svr.listen("0.0.0.0", port);
}

// ---------------- Main ----------------
static void usage(const char *argv0) {
  fprintf(stderr,
    "Usage: %s [--config PATH] [--no-webui] [--webui-port N]\n"
    "          --v4l2-dev /dev/video0 [--bgr] [--modeset=match-input]\n"
    "          [--present=stretch|fit|1:1] [--edid PATH] [--threads N]\n"
    "          [--crosshair DIAM or WxH] [--crosshair-center X,Y]\n"
    "          [--crosshair-mode=invert|solid] [--crosshair-color R,G,B]\n"
    "          [--crosshair-thickness N]\n",
    argv0
  );
}

int main(int argc, char **argv) {
  g_pid.store(getpid());
  enableRawModeNonBlockingStdin();

  bool webui_enabled = true;
  int webui_port = 8080;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
      g_cfg_path = argv[++i];
    }
  }

  {
    persisted_config loaded;
    loaded.xh.enabled = false;
    loaded.xh.diam_w = 0;
    loaded.xh.diam_h = 0;
    loaded.xh.thickness = 1;
    loaded.xh.mode = XH_INVERT;
    loaded.xh.solid_r = 255; loaded.xh.solid_g = 255; loaded.xh.solid_b = 255;

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

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--config") == 0) { i++; continue; }
    if (strcmp(argv[i], "--no-webui") == 0) { webui_enabled = false; continue; }
    if (strcmp(argv[i], "--webui-port") == 0 && i + 1 < argc) { webui_port = atoi(argv[++i]); continue; }

    std::lock_guard<std::mutex> lk(g_cfg_mtx);

    if (strcmp(argv[i], "--v4l2-dev") == 0 && i + 1 < argc) {
      g_cfg.v4l2_dev = argv[++i];
    } else if (strcmp(argv[i], "--modeset=match-input") == 0) {
      g_cfg.do_modeset = true;
    } else if (strcmp(argv[i], "--bgr") == 0) {
      g_cfg.input_is_bgr = true;
    } else if (strncmp(argv[i], "--present=", 10) == 0) {
      std::string p = argv[i] + 10;
      if (!present_policy_valid(p)) { fprintf(stderr, "Bad --present\n"); return 1; }
      g_cfg.present_policy = p;
    } else if (strcmp(argv[i], "--edid") == 0 && i + 1 < argc) {
      g_cfg.edid_path = argv[++i];
    } else if (strcmp(argv[i], "--threads") == 0 && i + 1 < argc) {
      g_cfg.threads = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--crosshair") == 0 && i + 1 < argc) {
      uint32_t w=0,h=0;
      if (!parse_dim(argv[++i], &w, &h)) { fprintf(stderr, "Bad --crosshair\n"); return 1; }
      g_cfg.xh.enabled = true;
      g_cfg.xh.diam_w = w; g_cfg.xh.diam_h = h;
    } else if (strcmp(argv[i], "--crosshair-center") == 0 && i + 1 < argc) {
      int32_t cx=0, cy=0;
      if (!parse_point_csv(argv[++i], &cx, &cy)) { fprintf(stderr, "Bad --crosshair-center\n"); return 1; }
      g_cfg.xh.enabled = true;
      g_cfg.xh.center_set = true;
      g_cfg.xh.cx = cx; g_cfg.xh.cy = cy;
    } else if (strncmp(argv[i], "--crosshair-mode=", 17) == 0) {
      std::string m = argv[i] + 17;
      g_cfg.xh.enabled = true;
      if (m == "invert") g_cfg.xh.mode = XH_INVERT;
      else if (m == "solid") g_cfg.xh.mode = XH_SOLID;
      else { fprintf(stderr, "Bad --crosshair-mode\n"); return 1; }
    } else if (strcmp(argv[i], "--crosshair-color") == 0 && i + 1 < argc) {
      uint8_t r=0,g=0,b=0;
      if (!parse_rgb_csv(argv[++i], &r, &g, &b)) { fprintf(stderr, "Bad --crosshair-color\n"); return 1; }
      g_cfg.xh.enabled = true;
      g_cfg.xh.mode = XH_SOLID;
      g_cfg.xh.solid_r=r; g_cfg.xh.solid_g=g; g_cfg.xh.solid_b=b;
    } else if (strcmp(argv[i], "--crosshair-thickness") == 0 && i + 1 < argc) {
      uint32_t t=0;
      if (!parse_u32(argv[++i], &t) || t==0 || t>99) { fprintf(stderr, "Bad --crosshair-thickness\n"); return 1; }
      g_cfg.xh.enabled = true;
      g_cfg.xh.thickness = t;
    } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      usage(argv[0]);
      return 0;
    } else {
      fprintf(stderr, "Unknown arg: %s\n", argv[i]);
      usage(argv[0]);
      return 1;
    }
  }

  {
    std::lock_guard<std::mutex> lk(g_cfg_mtx);
    (void)save_config_locked();
  }

  if (webui_enabled) {
    std::thread([=]() { webui_thread_main(webui_port); }).detach();
    fprintf(stderr, "[webui] open http://<device-ip>:%d/\n", webui_port);
  }

  pipeline p;
  bool have_pipeline = false;

  while (!g_quit.load()) {
    char ch;
    ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n == 1 && ch == 'q') { g_quit.store(true); break; }

    if (!have_pipeline) {
      auto snap = cfg_snapshot();
      if (!present_policy_valid(snap.present_policy)) snap.present_policy = "stretch";
      if (!p.init_from_config(snap)) {
        fprintf(stderr, "[supervisor] pipeline init failed; retrying in 1s...\n");
        p.shutdown();
        have_pipeline = false;
        usleep(1000 * 1000);
        continue;
      }
      have_pipeline = true;
      g_reinit_requested.store(false);
      fprintf(stderr, "[supervisor] pipeline running.\n");
    }

    if (g_reinit_requested.load()) {
      std::string reason = take_reinit_reason();
      fprintf(stderr, "[supervisor] reinit requested: %s\n", reason.c_str());
      p.shutdown();
      have_pipeline = false;
      g_reinit_requested.store(false);
      continue;
    }

    auto snap = cfg_snapshot();
    if (!p.step_frame(snap)) {
      fprintf(stderr, "[supervisor] pipeline step failed; reinitializing...\n");
      p.shutdown();
      have_pipeline = false;
      continue;
    }
  }

  fprintf(stderr, "[main] shutting down...\n");
  p.shutdown();
  return 0;
}

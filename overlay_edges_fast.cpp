#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <vector>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/fb.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
  #include <arm_neon.h>
  #define HAVE_NEON 1
#else
  #define HAVE_NEON 0
#endif

// RGB1555 helpers (fb0 overlay)
static inline uint16_t rgb1555(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t R = (uint16_t)(r >> 3);
  uint16_t G = (uint16_t)(g >> 3);
  uint16_t B = (uint16_t)(b >> 3);
  return (uint16_t)(0x8000 | (R << 10) | (G << 5) | (B << 0));
}

static inline int iabs(int v) { return v < 0 ? -v : v; }

// ------------------------ FB + /dev/mem mapping ------------------------
struct FbMap {
  int fd = -1;
  fb_var_screeninfo vinfo{};
  fb_fix_screeninfo finfo{};
  uint8_t *ptr = nullptr;
  size_t bytes = 0;

  bool open_map(const char *dev) {
    fd = ::open(dev, O_RDWR);
    if (fd < 0) { perror("open fb"); return false; }
    if (ioctl(fd, FBIOGET_VSCREENINFO, &vinfo)) { perror("FBIOGET_VSCREENINFO"); return false; }
    if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo)) { perror("FBIOGET_FSCREENINFO"); return false; }

    if (vinfo.bits_per_pixel != 16) {
      fprintf(stderr, "Expected 16bpp fb overlay; got %u\n", vinfo.bits_per_pixel);
      return false;
    }

    bytes = (size_t)vinfo.yres_virtual * (size_t)finfo.line_length;
    ptr = (uint8_t*)mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) { perror("mmap fb"); ptr = nullptr; return false; }
    return true;
  }

  void close_unmap() {
    if (ptr) munmap(ptr, bytes);
    if (fd >= 0) close(fd);
    ptr = nullptr; fd = -1; bytes = 0;
  }

  inline uint16_t* row_u16(int y) {
    return (uint16_t*)(ptr + (size_t)y * (size_t)finfo.line_length);
  }
};

struct MemMapRO {
  int fd = -1;
  uint8_t *map = nullptr;
  size_t map_len = 0;
  uint8_t *ptr = nullptr;

  bool map_phys(uint32_t phys, size_t len) {
    fd = ::open("/dev/mem", O_RDONLY);
    if (fd < 0) { perror("open /dev/mem"); return false; }

    long pagesz = sysconf(_SC_PAGESIZE);
    if (pagesz <= 0) pagesz = 4096;

    off_t base = (off_t)phys;
    off_t page_base = base & ~((off_t)pagesz - 1);
    off_t page_off  = base - page_base;

    map_len = (size_t)page_off + len;
    map = (uint8_t*)mmap(nullptr, map_len, PROT_READ, MAP_SHARED, fd, page_base);
    if (map == MAP_FAILED) { perror("mmap /dev/mem"); map = nullptr; return false; }
    ptr = map + page_off;
    return true;
  }

  void unmap_close() {
    if (map) munmap(map, map_len);
    if (fd >= 0) close(fd);
    map = nullptr; ptr = nullptr; fd = -1; map_len = 0;
  }
};

// ------------------------ Coarse edge pipeline (fast) ------------------------
// We do NOT run full sobel. We:
// 1) subsample Y into a coarse grid (no averaging)
// 2) compute cheap gradient magnitude: |dx|+|dy|
// 3) threshold => mask_c (0/1) on coarse grid
// 4) diff against previous coarse mask and only update changed blocks in fb0

static void build_luma_coarse_subsample_roi(const uint8_t *Y, int stride,
                                            int x0, int y0, int w, int h,
                                            int down,
                                            std::vector<uint8_t> &lum_c,
                                            int &wc, int &hc)
{
  if (down < 1) down = 1;
  wc = w / down;
  hc = h / down;

  if (wc < 3 || hc < 3) {
    wc = std::max(0, wc);
    hc = std::max(0, hc);
    lum_c.assign((size_t)wc * (size_t)hc, 0);
    return;
  }

  lum_c.assign((size_t)wc * (size_t)hc, 0);

  int off = down / 2; // sample center of each block
  for (int cy = 0; cy < hc; cy++) {
    int sy = y0 + cy * down + off;
    uint8_t *dst = &lum_c[(size_t)cy * (size_t)wc];

    const uint8_t *row = Y + sy * stride;
    for (int cx = 0; cx < wc; cx++) {
      int sx = x0 + cx * down + off;
      dst[cx] = row[sx];
    }
  }
}

static void coarse_gradient_mask(const std::vector<uint8_t> &lum_c, int wc, int hc,
                                 int thr, std::vector<uint8_t> &mask_c)
{
  mask_c.assign((size_t)wc * (size_t)hc, 0);
  if (wc < 3 || hc < 3) return;

  // borders remain 0
  for (int y = 1; y < hc - 1; y++) {
    const uint8_t *r0 = &lum_c[(size_t)(y - 1) * (size_t)wc];
    const uint8_t *r1 = &lum_c[(size_t)(y + 0) * (size_t)wc];
    const uint8_t *r2 = &lum_c[(size_t)(y + 1) * (size_t)wc];
    uint8_t *out = &mask_c[(size_t)y * (size_t)wc];

    for (int x = 1; x < wc - 1; x++) {
      int dx = iabs((int)r1[x + 1] - (int)r1[x - 1]);
      int dy = iabs((int)r2[x + 0] - (int)r0[x + 0]);
      int m = dx + dy;              // 0..510
      out[x] = (m >= thr) ? 1 : 0;  // 0/1
    }
  }
}

static inline void fb_fill_block_rgb1555(FbMap &fb,
                                         int x0, int y0, int bw, int bh,
                                         uint16_t pix)
{
  for (int y = 0; y < bh; y++) {
    uint16_t *row = fb.row_u16(y0 + y);
    for (int x = 0; x < bw; x++) {
      row[x0 + x] = pix;
    }
  }
}

// Diff coarse mask and only update changed coarse cells by drawing blocks.
// ROI: x0,y0,w,h are in full-res pixel coordinates.
// down: coarse cell size in pixels.
static void diff_write_mask_coarse_blocks_roi(FbMap &fb,
                                              const std::vector<uint8_t> &mask_now_c,
                                              std::vector<uint8_t> &mask_prev_c,
                                              int wc, int hc,
                                              int x0, int y0, int w, int h,
                                              int down,
                                              uint16_t on_px, uint16_t off_px)
{
  if ((int)mask_prev_c.size() != wc * hc) mask_prev_c.assign((size_t)wc * (size_t)hc, 0);

  // Full blocks only (we build coarse grid from w/down, h/down)
  const int bw = down;
  const int bh = down;

  for (int cy = 0; cy < hc; cy++) {
    const uint8_t *now = &mask_now_c[(size_t)cy * (size_t)wc];
    uint8_t *prev = &mask_prev_c[(size_t)cy * (size_t)wc];

    int py = y0 + cy * down;
    for (int cx = 0; cx < wc; cx++) {
      uint8_t nn = now[cx];
      uint8_t pp = prev[cx];
      if (nn == pp) continue;
      prev[cx] = nn;

      int px = x0 + cx * down;

      // Safety (should already be safe if ROI is aligned and within screen)
      if (px < 0 || py < 0) continue;
      if (px + bw > x0 + w) continue;
      if (py + bh > y0 + h) continue;

      fb_fill_block_rgb1555(fb, px, py, bw, bh, nn ? on_px : off_px);
    }
  }
}

// ------------------------ CLI + main loop ------------------------
static void usage(const char *p) {
  printf("Usage: %s [options]\n", p);
  printf("  -d <fbdev>     overlay fb (default /dev/fb0)\n");
  printf("  -V <phys_hex>  NV12 base phys (default 0x8BBE8000)\n");
  printf("  -s <stride>    NV12 stride (default = fb width)\n");
  printf("  -t <thr>       coarse edge threshold (0..510-ish, default 100)\n");
  printf("  -C <hex>       clear pixel RGB1555 hex (default 0000)\n");
  printf("  -D <N>         downscale cell size (default 8, allowed 1..32)\n");
  printf("  -F <ms>        frame delay ms (default 66 ~15fps)\n");
  printf("  -R             right-half mode (process only x=640..1279)\n");
  printf("  -B <top,bottom> ignore black borders: skip y in [0,top) and [h-bottom,h)\n");
}

int main(int argc, char **argv) {
  const char *fbdev = "/dev/fb0";
  uint32_t base = 0x8BBE8000;
  int stride = 0;

  int thr = 100;            // note: coarse dx+dy => tune higher than old sobel thresholds
  uint16_t clear_px = 0x0000;
  int down = 8;             // cell size in pixels, 1..32
  int frame_delay_ms = 66;  // ~15fps

  bool right_half = false;
  int border_top = 0;
  int border_bottom = 0;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-d") && i+1 < argc) { fbdev = argv[++i]; continue; }
    if (!strcmp(argv[i], "-V") && i+1 < argc) { base = (uint32_t)strtoul(argv[++i], nullptr, 0); continue; }
    if (!strcmp(argv[i], "-s") && i+1 < argc) { stride = atoi(argv[++i]); continue; }
    if (!strcmp(argv[i], "-t") && i+1 < argc) { thr = atoi(argv[++i]); continue; }
    if (!strcmp(argv[i], "-C") && i+1 < argc) { clear_px = (uint16_t)strtoul(argv[++i], nullptr, 16); continue; }
    if (!strcmp(argv[i], "-D") && i+1 < argc) { down = atoi(argv[++i]); continue; }
    if (!strcmp(argv[i], "-F") && i+1 < argc) { frame_delay_ms = atoi(argv[++i]); continue; }
    if (!strcmp(argv[i], "-R")) { right_half = true; continue; }
    if (!strcmp(argv[i], "-B") && i+1 < argc) {
      const char *s = argv[++i];
      // parse "top,bottom" with minimal libc (no sscanf assumptions)
      border_top = atoi(s);
      const char *comma = strchr(s, ',');
      border_bottom = comma ? atoi(comma + 1) : 0;
      continue;
    }
    if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(argv[0]); return 0; }

    fprintf(stderr, "Unknown arg: %s\n", argv[i]);
    usage(argv[0]);
    return 1;
  }

  if (down < 1) down = 1;
  if (down > 32) down = 32;
  if (frame_delay_ms < 0) frame_delay_ms = 0;
  if (thr < 0) thr = 0;

  FbMap fb;
  if (!fb.open_map(fbdev)) return 1;

  const int fb_w = (int)fb.vinfo.xres;
  const int fb_h = (int)fb.vinfo.yres;
  if (stride <= 0) stride = fb_w;

  // ROI selection (full-res coordinates)
  int roi_x0 = 0;
  int roi_w  = fb_w;
  if (right_half) {
    roi_x0 = fb_w / 2;   // 640 for 1280
    roi_w  = fb_w - roi_x0;
  }

  int roi_y0 = std::max(0, border_top);
  int roi_y1 = fb_h - std::max(0, border_bottom);
  if (roi_y1 < roi_y0) roi_y1 = roi_y0;
  int roi_h = roi_y1 - roi_y0;

  // Align ROI to 'down' so blocks fit neatly and we never touch outside desired region.
  // (This also ensures we truly don't overwrite borders.)
  auto align_up = [](int v, int a) { return (v + a - 1) / a * a; };
  auto align_dn = [](int v, int a) { return v / a * a; };

  int ax0 = align_up(roi_x0, down);
  int ay0 = align_up(roi_y0, down);
  int ax1 = align_dn(roi_x0 + roi_w, down);
  int ay1 = align_dn(roi_y0 + roi_h, down);

  if (ax1 < ax0) ax1 = ax0;
  if (ay1 < ay0) ay1 = ay0;

  roi_x0 = ax0;
  roi_y0 = ay0;
  roi_w  = ax1 - ax0;
  roi_h  = ay1 - ay0;

  // Map NV12 frame (we only read Y plane; size still includes UV but we don't touch it).
  const size_t frame_bytes = (size_t)stride * (size_t)fb_h * 3 / 2;
  MemMapRO nv;
  if (!nv.map_phys(base, frame_bytes)) return 1;
  const uint8_t *Y = nv.ptr; // NV12 Y plane begins at base

  // Solid red overlay color
  const uint16_t red_px = rgb1555(255, 0, 0);

  // Coarse buffers
  std::vector<uint8_t> lum_c;
  std::vector<uint8_t> mask_c;
  std::vector<uint8_t> mask_prev_c;
  int wc = 0, hc = 0;

  printf("FB: %s %dx%d fb_stride=%u bpp=%u\n", fbdev, fb_w, fb_h, fb.finfo.line_length, fb.vinfo.bits_per_pixel);
  printf("NV12: base=0x%08x stride=%d frame=0x%zx\n", base, stride, frame_bytes);
  printf("Mode: ROI x=%d..%d y=%d..%d (w=%d h=%d) down=%d thr=%d\n",
         roi_x0, roi_x0 + roi_w, roi_y0, roi_y0 + roi_h, roi_w, roi_h, down, thr);
  printf("Clear: 0x%04x  Color: RED  Delay: %d ms\n", clear_px, frame_delay_ms);

  // Always loop
  for (;;) {
    // Build coarse luma from ROI only
    build_luma_coarse_subsample_roi(Y, stride, roi_x0, roi_y0, roi_w, roi_h, down, lum_c, wc, hc);

    // Compute coarse edge mask on coarse grid
    coarse_gradient_mask(lum_c, wc, hc, thr, mask_c);

    // Diff + update only changed blocks, writing only inside ROI
    diff_write_mask_coarse_blocks_roi(fb, mask_c, mask_prev_c, wc, hc,
                                      roi_x0, roi_y0, roi_w, roi_h, down,
                                      red_px, clear_px);

    if (frame_delay_ms) usleep((useconds_t)frame_delay_ms * 1000);
  }

  // unreachable
  nv.unmap_close();
  fb.close_unmap();
  return 0;
}

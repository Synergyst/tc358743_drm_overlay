#pragma once
#include <stdint.h>
#include <string.h>

static inline uint8_t clamp_u8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : (uint8_t)v); }

static inline uint32_t pack_xrgb8888(uint8_t r, uint8_t g, uint8_t b) {
  return 0xFF000000u | ((uint32_t)r<<16) | ((uint32_t)g<<8) | (uint32_t)b;
}

// Basic BT.601 integer conversion.
static inline void yuv_to_rgb(int y, int u, int v, uint8_t *r, uint8_t *g, uint8_t *b) {
  int C = y - 16;
  int D = u - 128;
  int E = v - 128;
  int Rt = (298*C + 409*E + 128) >> 8;
  int Gt = (298*C - 100*D - 208*E + 128) >> 8;
  int Bt = (298*C + 516*D + 128) >> 8;
  *r = clamp_u8(Rt);
  *g = clamp_u8(Gt);
  *b = clamp_u8(Bt);
}

static inline void convert_rgb24_to_xrgb8888(
  const uint8_t *src, uint32_t src_stride,
  uint32_t *dst, uint32_t dst_stride_bytes,
  uint32_t w, uint32_t h,
  bool bgr
) {
  for (uint32_t y=0; y<h; y++) {
    const uint8_t *s = src + (uint64_t)y * src_stride;
    uint32_t *d = (uint32_t*)((uint8_t*)dst + (uint64_t)y * dst_stride_bytes);
    for (uint32_t x=0; x<w; x++) {
      const uint8_t *p = s + x*3u;
      uint8_t r = bgr ? p[2] : p[0];
      uint8_t g = p[1];
      uint8_t b = bgr ? p[0] : p[2];
      d[x] = pack_xrgb8888(r,g,b);
    }
  }
}

// YUYV: Y0 U Y1 V
static inline void convert_yuyv_to_xrgb8888(
  const uint8_t *src, uint32_t src_stride,
  uint32_t *dst, uint32_t dst_stride_bytes,
  uint32_t w, uint32_t h
) {
  for (uint32_t y=0; y<h; y++) {
    const uint8_t *s = src + (uint64_t)y * src_stride;
    uint32_t *d = (uint32_t*)((uint8_t*)dst + (uint64_t)y * dst_stride_bytes);
    for (uint32_t x=0; x+1<w; x+=2) {
      int Y0=s[0], U=s[1], Y1=s[2], V=s[3];
      uint8_t r,g,b;
      yuv_to_rgb(Y0,U,V,&r,&g,&b); d[x+0] = pack_xrgb8888(r,g,b);
      yuv_to_rgb(Y1,U,V,&r,&g,&b); d[x+1] = pack_xrgb8888(r,g,b);
      s += 4;
    }
    if (w & 1) {
      int Y0=s[0], U=s[1], V=s[3];
      uint8_t r,g,b;
      yuv_to_rgb(Y0,U,V,&r,&g,&b); d[w-1] = pack_xrgb8888(r,g,b);
    }
  }
}

// UYVY: U Y0 V Y1
static inline void convert_uyvy_to_xrgb8888(
  const uint8_t *src, uint32_t src_stride,
  uint32_t *dst, uint32_t dst_stride_bytes,
  uint32_t w, uint32_t h
) {
  for (uint32_t y=0; y<h; y++) {
    const uint8_t *s = src + (uint64_t)y * src_stride;
    uint32_t *d = (uint32_t*)((uint8_t*)dst + (uint64_t)y * dst_stride_bytes);
    for (uint32_t x=0; x+1<w; x+=2) {
      int U=s[0], Y0=s[1], V=s[2], Y1=s[3];
      uint8_t r,g,b;
      yuv_to_rgb(Y0,U,V,&r,&g,&b); d[x+0] = pack_xrgb8888(r,g,b);
      yuv_to_rgb(Y1,U,V,&r,&g,&b); d[x+1] = pack_xrgb8888(r,g,b);
      s += 4;
    }
    if (w & 1) {
      int U=s[0], Y0=s[1], V=s[2];
      uint8_t r,g,b;
      yuv_to_rgb(Y0,U,V,&r,&g,&b); d[w-1] = pack_xrgb8888(r,g,b);
    }
  }
}

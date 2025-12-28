#pragma once
#include <linux/videodev2.h>
#include <string>
#include <vector>

struct v4l2_format_caps {
  uint32_t pixfmt = 0;          // V4L2 fourcc
  std::string desc;            // human description
  std::vector<std::pair<uint32_t,uint32_t>> sizes; // WxH (discrete only for now)
};

struct v4l2_device_caps {
  std::string dev;
  bool ok = false;
  std::string err;
  std::vector<v4l2_format_caps> formats;
};

static inline std::string fourcc_to_string(uint32_t f) {
  char s[5];
  s[0] = (char)(f & 0xFF);
  s[1] = (char)((f >> 8) & 0xFF);
  s[2] = (char)((f >> 16) & 0xFF);
  s[3] = (char)((f >> 24) & 0xFF);
  s[4] = 0;
  return std::string(s);
}

static inline uint32_t string_to_fourcc(const std::string &s) {
  if (s.size() != 4) return 0;
  return (uint32_t)(uint8_t)s[0] |
         ((uint32_t)(uint8_t)s[1] << 8) |
         ((uint32_t)(uint8_t)s[2] << 16) |
         ((uint32_t)(uint8_t)s[3] << 24);
}

// Implemented in v4l2_caps.cpp
v4l2_device_caps v4l2_query_caps(const std::string &dev);

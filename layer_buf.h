#ifndef LAYER_BUF_H
#define LAYER_BUF_H

#include <cstdint>
#include <vector>

struct layer_buf {
  uint32_t w = 0, h = 0;
  std::vector<uint32_t> px;

  void resize(uint32_t W, uint32_t H) {
    w = W; h = H;
    px.resize((size_t)w * (size_t)h);
  }
};

#endif

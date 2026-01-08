#pragma once
#include <cstdint>
#include <vector>
#include <mutex>

struct GpuCLAHE {
    int W = 0, H = 0;
    int xTiles = 0, yTiles = 0;
    int tileW = 0, tileH = 0;

    // GL/EGL opaque handles (kept in cpp)
    void* dpy = nullptr;
    void* ctx = nullptr;

    void reset();

    // GL objects
    unsigned progHist = 0;
    unsigned progClip = 0;
    unsigned progCdf  = 0;
    unsigned progApply= 0;
    unsigned progClearHist = 0;

    unsigned texLuma = 0;       // R8
    unsigned texOut  = 0;       // R8
    unsigned ssboHist = 0;      // uint hist[tileCount][256]
    unsigned ssboCdf  = 0;      // uint cdf[tileCount][256]

    unsigned readFbo = 0;

    // staging for upload/download (ROI not needed for full-frame CLAHE)
    std::vector<uint8_t> outLuma;

    std::mutex mtx;
    bool ok = false;

    bool init(int w, int h, int x_tiles, int y_tiles);
    bool run(const uint8_t* inLuma, float clip_limit, uint8_t* outLuma);

    ~GpuCLAHE();
};

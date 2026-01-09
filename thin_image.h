#pragma once

/*
 * C++ translation of:
 * "Efficient Binary Image Thinning using Neighborhood Maps"
 * Joseph M. Cychosz, Graphics Gems IV, 1994
 *
 * Expectations:
 *   - data is an 8-bit binary image: 0 = background, non-zero = foreground
 *   - thinning is done in-place
 *   - width/height are pixel dimensions
 *   - strideBytes is bytes per row (>= width)
 *
 * This is a faithful port of the original control flow/semantics.
 */

#include <cstdint>
#include <cstdlib>   // std::malloc, std::free
#include <cstring>   // std::memset
#include <functional>

namespace thinimg {

using ThinProgressFn = std::function<void(int /*pass*/, int /*deleted*/)>;

inline int ThinImageBinaryInPlace(
    std::uint8_t* data,
    int width,
    int height,
    int strideBytes,
    ThinProgressFn progress = nullptr
) {
    if (!data || width <= 0 || height <= 0 || strideBytes < width) return 0;

    // Direction masks: N, S, W, E
    static constexpr int masks[4] = { 0200, 0002, 0040, 0010 };

    // delete[512] table from the original code (unchanged)
    static constexpr std::uint8_t del[512] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };

    auto pix = [&](int x, int y) -> std::uint8_t& {
        return *(data + (std::size_t)y * (std::size_t)strideBytes + (std::size_t)x);
    };

    // qb holds neighborhood maps for previous scanline positions (as in original)
    std::uint8_t* qb = (std::uint8_t*)std::malloc((std::size_t)width * sizeof(std::uint8_t));
    if (!qb) return 0;
    qb[width - 1] = 0; // used for lower-right pixel

    int pc = 0;
    int count = 1;

    while (count) {
        pc++;
        count = 0;

        for (int i = 0; i < 4; i++) {
            const int m = masks[i];

            // Build initial previous scan buffer from scanline 0
            int p = pix(0, 0) != 0;
            for (int x = 0; x < width - 1; x++) {
                qb[x] = (std::uint8_t)(p = ((p << 1) & 0006) | (pix(x + 1, 0) != 0));
            }

            // Scan image for pixel deletion candidates
            for (int y = 0; y < height - 1; y++) {
                int q = qb[0];
                p = ((q << 3) & 0110) | (pix(0, y + 1) != 0);

                for (int x = 0; x < width - 1; x++) {
                    q = qb[x];
                    p = ((p << 1) & 0666) | ((q << 3) & 0110) | (pix(x + 1, y + 1) != 0);
                    qb[x] = (std::uint8_t)p;

                    if (((p & m) == 0) && del[p]) {
                        count++;
                        pix(x, y) = 0;
                    }
                }

                // Process right edge pixel
                p = (p << 1) & 0666;
                if (((p & m) == 0) && del[p]) {
                    count++;
                    pix(width - 1, y) = 0;
                }
            }

            // Process bottom scan line
            for (int x = 0; x < width; x++) {
                int q = qb[x];
                p = ((p << 1) & 0666) | ((q << 3) & 0110);
                if (((p & m) == 0) && del[p]) {
                    count++;
                    pix(x, height - 1) = 0;
                }
            }
        }

        if (progress) progress(pc, count);
    }

    std::free(qb);
    return pc;
}

} // namespace thinimg

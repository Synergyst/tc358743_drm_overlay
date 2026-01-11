#ifndef HUMAN_OUTLINE_H
#define HUMAN_OUTLINE_H

#include "layer_buf.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>
#include <algorithm>

class HumanOutlineProcessor {
private:
    cv::Ptr<cv::CLAHE> clahe;
    cv::Mat gray, eq, work, edges, bin;

    static int clampInt(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

    // Map 0..255 -> odd kernel size 1..31
    static int blurKsizeFromStrength(uint8_t blur_strength) {
        if (blur_strength == 0) return 1;
        int k = 1 + (blur_strength / 8) * 2; // 0->1, 8->3, 16->5 ...
        k = clampInt(k, 1, 31);
        if ((k & 1) == 0) k += 1;
        return k;
    }

public:
    HumanOutlineProcessor() {
        clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    }

    /**
     * Reusing your external variable names/limits:
     *  - gain: 0.01..255 (higher = more sensitive)
     *  - threshold: 0..255
     *  - blur_strength: 0..255
     *  - threads: 0..4 (0=auto; else 1..4)
     */
    void process(layer_buf& buf,
                 float gain = 3.0f,
                 uint8_t threshold = 8,
                 uint8_t blur_strength = 1,
                 uint8_t threads = 1,
                 int thickness = 2)
    {
        if (buf.px.empty() || buf.w == 0 || buf.h == 0) return;

        if (threads != 0) {
            cv::setNumThreads((int)std::min<uint8_t>(threads, 4));
        }

        cv::Mat matFrame((int)buf.h, (int)buf.w, CV_8UC4, buf.px.data());

        // 1) Gray
        cv::cvtColor(matFrame, gray, cv::COLOR_BGRA2GRAY);

        // 2) Contrast normalize (very helpful on thermal)
        clahe->apply(gray, eq);

        // 3) Blur (denoise)
        const int k = blurKsizeFromStrength(blur_strength);
        if (k > 1) cv::GaussianBlur(eq, work, cv::Size(k, k), 0.0);
        else work = eq;

        // 4) Morphological gradient (outline-like edges)
        // Kernel size influenced by gain slightly (kept bounded).
        int kg = 3 + (int)(std::min(20.0f, std::max(0.01f, gain)) / 5.0f); // ~3..7
        if ((kg & 1) == 0) kg += 1;
        kg = clampInt(kg, 3, 9);

        cv::Mat kgrad = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kg, kg));
        cv::morphologyEx(work, edges, cv::MORPH_GRADIENT, kgrad);

        // Optional: mix in Canny for crispness when gain is high.
        // Keep it cheap: Canny on the already-smoothed image.
        if (gain >= 2.0f) {
            cv::Mat can;
            double lo = std::max(5.0, 20.0 / std::max(0.01f, gain));
            double hi = lo * 3.0;
            cv::Canny(work, can, lo, hi, 3, true);
            cv::bitwise_or(edges, can, edges);
        }

        // 5) Threshold -> binary mask
        // Interpret threshold as "edge strength cutoff" (0..255).
        // If user leaves threshold very low, auto-raise a bit based on gain to avoid full-white.
        int t = (int)threshold;
        if (t == 0) {
            t = clampInt((int)(8 + gain * 2.0f), 4, 64);
        }
        cv::threshold(edges, bin, (double)t, 255.0, cv::THRESH_BINARY);

        // 6) Cleanup (remove specks, connect lines)
        cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(bin, bin, cv::MORPH_OPEN, k3);
        cv::morphologyEx(bin, bin, cv::MORPH_DILATE, k3);

        // 7) Contours -> draw on original frame
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Green outline on top of the original image (matches your prior idea)
        cv::drawContours(matFrame, contours, -1, cv::Scalar(0, 255, 0, 255), thickness);
    }
};

#endif

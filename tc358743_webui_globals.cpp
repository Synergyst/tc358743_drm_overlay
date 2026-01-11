#include "tc358743_webui.h"

std::mutex g_frame_mtx;
std::condition_variable g_frame_cv;
std::vector<uint8_t> g_current_mjpeg_frame;
uint64_t g_frame_sequence = 0;

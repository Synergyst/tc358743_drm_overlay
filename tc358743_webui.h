#pragma once
#include <atomic>
#include <string>
#include <vector>
#include <condition_variable>

// Existing API wiring
void webui_set_config_json_provider(std::string (*fn)());
void webui_set_apply_handler(bool (*fn)(const std::string&, std::string&, int&));
void webui_set_status_provider(std::string (*fn)());
void webui_set_quit_flag(std::atomic<bool> *quit_flag);
void webui_set_reference_png(const std::vector<uint8_t> &png_bytes);
void webui_set_listen_address(const std::string &addr);

// Filter registry provider (dynamic enumeration)
void webui_set_filter_defs_provider(std::string (*fn)());
// Backward-compatible alias for older code.
void webui_set_filters_provider(std::string (*fn)());

// NEW: V4L2 capabilities provider.
// Called for GET /api/v4l2/caps?dev=/dev/videoX
// Must return a JSON string.
void webui_set_v4l2_caps_provider(std::string (*fn)(const std::string &dev));

// Start server thread
void webui_start_detached(int port);

// Live feed
extern std::mutex g_frame_mtx;
extern std::condition_variable g_frame_cv;
extern std::vector<uint8_t> g_current_mjpeg_frame;
extern uint64_t g_frame_sequence; // To track if a frame is actually "new"

#pragma once
#include <atomic>
#include <string>
#include <vector>

// Wiring from main program:
// Provide config as JSON text (keeps persisted_config opaque to this TU)
void webui_set_config_json_provider(std::string (*fn)());
void webui_set_apply_handler(bool (*fn)(const std::string &body, std::string &out_json, int &out_http_status));
void webui_set_status_provider(std::string (*fn)());
void webui_set_quit_flag(std::atomic<bool> *quit_flag);
void webui_set_reference_png(const std::vector<uint8_t> &png_bytes);
void webui_set_listen_address(const std::string &addr);

// Optional: provide supported filter definitions (as JSON) for dynamic UI enumeration.
// If not set, the UI falls back to a built-in list (mono + sobel).
// Expected JSON shape example:
// {
//   "filters":[
//     {"id":"mono","name":"Monochrome","params":[{"k":"mode","type":"enum","values":["luma"]}]},
//     {"id":"sobel","name":"Sobel","params":[{"k":"mode","type":"enum","values":["transparentNonEdges","grayscale"]}]}
//   ]
// }
void webui_set_filter_defs_provider(std::string (*fn)());

// Start server:
void webui_start_detached(int port);

// Backward-compatible alias (older backend code may call this name).
// Same semantics as webui_set_filter_defs_provider().
void webui_set_filters_provider(std::string (*fn)());

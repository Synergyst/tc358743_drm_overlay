#pragma once
#include <stdint.h>
#include <string>
#include <vector>
#include "third_party/json.hpp"

struct rect_u32 { uint32_t x=0,y=0,w=0,h=0; };
struct point_i32 { int32_t x=0,y=0; };

enum layer_invert_rel { INV_NONE=0, INV_LOWER=1, INV_UPPER=2 };
enum layer_type { LAYER_VIDEO=0, LAYER_CROSSHAIR=1, LAYER_GRAPHICS=2 };

enum filter_id { FILTER_MONO=1, FILTER_SOBEL=2, FILTER_DENOISE=3, FILTER_RGB_MAP=4, FILTER_RGB_KEY_ALPHA=5, FILTER_M3LITE_EDGE_MASK=6, FILTER_CLAHE=7, FILTER_UNKNOWN=0 };

struct filter_cfg {
  filter_id id = FILTER_UNKNOWN;
  float mono_strength = 1.0f;

  enum sobel_mode_t { SOBEL_EDGES_ONLY=0, SOBEL_MAGNITUDE=1 } sobel_mode = SOBEL_EDGES_ONLY;
  uint8_t sobel_threshold = 64;
  float sobel_alpha = 1.0f;
  bool sobel_invert = false;

  // Denoise (simple box blur in layer space)
  int denoise_radius = 1;     // 1..8 (suggest)
  float denoise_strength = 1.0f; // 0..1 blend with original

  // RGB range -> RGB value mapping filter ("rgbMap")
  // Match: if min/max are -1, that channel is wildcard (always matches).
  // If matches, output pixel becomes (r_out, g_out, b_out) keeping alpha=255.
  int r_min = -1, r_max = -1; uint8_t r_out = 255;
  int g_min = -1, g_max = -1; uint8_t g_out = 255;
  int b_min = -1, b_max = -1; uint8_t b_out = 255;
  int a_min = -1, a_max = -1; uint8_t a_out = 255;

  // rgbKeyAlpha: convert RGB to a given value while keying near-black/near-white to a given alpha.
  // If "setRgb" is false, RGB is preserved; only alpha is modified.
  enum key_mode_t { KEY_BLACK=0, KEY_WHITE=1 } key_mode = KEY_BLACK;
  int key_threshold = 16;     // 0..255, how close to black/white counts as key
  uint8_t key_alpha = 0;      // alpha for keyed pixels
  uint8_t keep_alpha = 255;   // alpha for non-keyed pixels
  bool set_rgb = false;       // if true, set RGB to out_r/out_g/out_b (for all pixels)
  uint8_t out_r = 255, out_g = 255, out_b = 255;

  // CLAHE algorithm:
  uint8_t clahe_x_tiles = 8;
  uint8_t clahe_y_tiles = 8;
  float clahe_clip_limit = 2.0f;
};

struct crosshair_layer_cfg {
  bool enabled=false;
  uint32_t diam_w=50, diam_h=50;
  bool center_set=false;
  int32_t cx=0, cy=0;
  uint32_t thickness=1;
  bool solid=true;
  uint8_t r=255,g=255,b=255;
  float opacity=1.0f;
  layer_invert_rel invert_rel=INV_NONE;
};

// OAK per-layer role:
// - NONE: layer behaves normally
// - VISIBLE_IN: this layer's (layer-space) buffer is sent to OAK as "visible"
// - THERMAL_IN: this layer's (layer-space) buffer is sent to OAK as "thermal"
// - OUTPUT: this layer displays OAK output instead of its own sampled pixels
enum oak_role {
  OAK_NONE = 0,
  OAK_VISIBLE_IN = 1,
  OAK_THERMAL_IN = 2,
  OAK_OUTPUT = 3,
};

struct oak_layer_cfg {
  bool enabled = false;           // master switch for this layer participating in OAK pipeline
  oak_role role = OAK_NONE;       // visible/thermal/output
  // Future: warp params, thresholds, etc.
  // For v0, processing is fixed in oak_accel.cpp Script.
};

struct layer_cfg {
  std::string name="Layer";
  layer_type type=LAYER_VIDEO;
  bool enabled=true;
  std::string video_source; // per-layer source selection (device path), empty => use config.v4l2Dev
  rect_u32 src_rect{0,0,0,0};
  point_i32 dst_pos{0,0};
  float scale_x=1.0f;
  float scale_y=1.0f;
  float opacity=1.0f;
  layer_invert_rel invert_rel=INV_NONE;
  crosshair_layer_cfg xh{};
  std::vector<filter_cfg> filters{};

  // New: optional OAK usage
  oak_layer_cfg oak{};
};

struct viewport_cfg { bool set=false; uint32_t w=0,h=0; };

// Aux device configuration (generic V4L2)
struct aux_source_cfg {
  std::string dev;
  std::string mode = "auto"; // "auto" | "manual"
  uint32_t width = 0;
  uint32_t height = 0;
  std::string pixfmt = "";   // FourCC as 4-char string, e.g. "YUYV"
};

struct persisted_config {
  // Default layer source
  std::string v4l2_dev="/dev/v4l/by-path/platform-fe800000.csi-video-index0";

  // Hard-coded TC358743 pair but reorderable
  std::vector<std::string> tc358743_order{"/dev/v4l/by-path/platform-fe800000.csi-video-index0","/dev/v4l/by-path/platform-fe801000.csi-video-index0"};

  // Aux sources (generic V4L2)
  std::vector<aux_source_cfg> aux_sources{};

  // Backward compatibility (legacy)
  std::vector<std::string> v4l2_sources{};

  std::string edid_path;
  bool do_modeset=false;
  int threads=0;
  bool input_is_bgr=false;
  std::string present_policy="fit";
  bool prefer_output_mode=true;
  uint32_t prefer_out_w=1280;
  uint32_t prefer_out_h=720;
  viewport_cfg viewport{};
  std::vector<layer_cfg> layers{};
  std::string listen_addr="0.0.0.0";

  // New: OAK device selection and processing size.
  // If oak_enable is false, the program behaves exactly as before.
  bool oak_enable = false;
  std::string oak_mxid = "";       // optional specific device MxId, empty => first device
  uint32_t oak_w = 320;
  uint32_t oak_h = 180;
};

bool present_policy_valid(const std::string &p);

// Normalizes config in-place:
// - ensures tc358743_order has exactly 2 entries
// - ensures v4l2_dev is set
// - ensures legacy v4l2_sources is populated for UI compatibility
void cfg_normalize(persisted_config &c);

// Returns full effective source list (TC order + aux + legacy extras)
std::vector<std::string> cfg_effective_sources(const persisted_config &c);

// JSON serialization / parsing
std::string config_to_json(const persisted_config &c_in);
bool config_from_json_text(const std::string &text, persisted_config &c);

// Filter registry JSON (for /api/filters)
nlohmann::json filter_registry_json();

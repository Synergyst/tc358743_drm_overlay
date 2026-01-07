#include "overlay_backend.h"
#include <algorithm>
#include <sstream>
#include <fstream>

using nlohmann::json;

static bool parse_u32(const char *s, uint32_t *out) {
  if (!s || !*s) return false;
  char *end = NULL;
  errno = 0;
  unsigned long v = strtoul(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') return false;
  if (v > 0xFFFFFFFFul) return false;
  *out = (uint32_t)v;
  return true;
}

static bool parse_dim(const char *s, uint32_t *out_w, uint32_t *out_h) {
  const char *x = (s ? strchr(s, 'x') : NULL);
  if (!s || !*s) return false;
  if (!x) {
    uint32_t d = 0;
    if (!parse_u32(s, &d) || d == 0) return false;
    *out_w = d; *out_h = d;
    return true;
  }
  char a[64], b[64];
  size_t la = (size_t)(x - s);
  size_t lb = strlen(x + 1);
  if (la == 0 || lb == 0 || la >= sizeof(a) || lb >= sizeof(b)) return false;
  memcpy(a, s, la); a[la] = 0;
  memcpy(b, x + 1, lb); b[lb] = 0;
  uint32_t w = 0, h = 0;
  if (!parse_u32(a, &w) || !parse_u32(b, &h)) return false;
  if (w == 0 || h == 0) return false;
  *out_w = w; *out_h = h;
  return true;
}

static bool parse_rgb_csv(const char *s, uint8_t *r, uint8_t *g, uint8_t *b) {
  if (!s) return false;
  int rr=-1, gg=-1, bb=-1;
  if (sscanf(s, "%d,%d,%d", &rr, &gg, &bb) != 3) return false;
  if (rr<0||rr>255||gg<0||gg>255||bb<0||bb>255) return false;
  *r=(uint8_t)rr; *g=(uint8_t)gg; *b=(uint8_t)bb;
  return true;
}

static bool parse_point_csv(const char *s, int32_t *x, int32_t *y) {
  if (!s) return false;
  int xx=0, yy=0;
  if (sscanf(s, "%d,%d", &xx, &yy) != 2) return false;
  *x=(int32_t)xx; *y=(int32_t)yy;
  return true;
}

bool present_policy_valid(const std::string &p) {
  return p=="stretch" || p=="fit" || p=="1:1";
}

static void uniq_push(std::vector<std::string> &out, const std::string &s) {
  if (s.empty()) return;
  if (std::find(out.begin(), out.end(), s) == out.end()) out.push_back(s);
}

// ---------------- OAK helpers ----------------
static const char* oak_role_to_string(oak_role r) {
  switch(r) {
    case OAK_NONE:       return "none";
    case OAK_VISIBLE_IN: return "visible";
    case OAK_THERMAL_IN: return "thermal";
    case OAK_OUTPUT:     return "output";
    default:             return "none";
  }
}

static oak_role oak_role_from_string(const std::string& s) {
  if (s == "visible") return OAK_VISIBLE_IN;
  if (s == "thermal") return OAK_THERMAL_IN;
  if (s == "output")  return OAK_OUTPUT;
  return OAK_NONE;
}

void cfg_normalize(persisted_config &c) {
  // Ensure tc order
  std::vector<std::string> tc;
  for (auto &s : c.tc358743_order) uniq_push(tc, s);
  if (tc.size() < 2) {
    uniq_push(tc, "/dev/v4l/by-path/platform-fe800000.csi-video-index0");
    uniq_push(tc, "/dev/v4l/by-path/platform-fe801000.csi-video-index0");
  }
  if (tc.size() > 2) tc.resize(2);
  c.tc358743_order = tc;

  // Default v4l2Dev if empty
  if (c.v4l2_dev.empty()) c.v4l2_dev = c.tc358743_order[0];

  // Legacy v4l2Sources: keep it as a display list for the UI
  std::vector<std::string> legacy;
  for (auto &s : c.tc358743_order) uniq_push(legacy, s);
  for (auto &a : c.aux_sources) uniq_push(legacy, a.dev);
  for (auto &s : c.v4l2_sources) uniq_push(legacy, s); // keep old additions if present
  c.v4l2_sources = legacy;

  // Sanitize aux entries
  std::vector<aux_source_cfg> aux;
  for (auto &a : c.aux_sources) {
    if (a.dev.empty()) continue;
    if (a.mode != "manual") a.mode = "auto";
    if (a.mode == "manual") {
      if (a.width == 0 || a.height == 0) a.mode = "auto";
      if (!a.pixfmt.empty() && a.pixfmt.size() != 4) a.pixfmt.clear();
    } else {
      a.width = 0; a.height = 0; a.pixfmt.clear();
    }
    aux.push_back(a);
  }
  c.aux_sources = aux;

  // ---- OAK defaults / sanitization ----
  if (c.oak_w == 0) c.oak_w = 320;
  if (c.oak_h == 0) c.oak_h = 180;
  // Keep it bounded to something sane for your use-case
  if (c.oak_w > 1920) c.oak_w = 1920;
  if (c.oak_h > 1080) c.oak_h = 1080;

  // Per-layer oak fields: no heavy normalization, just clamp roles for non-video
  for (auto &L : c.layers) {
    if (L.type != LAYER_VIDEO) {
      L.oak.enabled = false;
      L.oak.role = OAK_NONE;
    }
  }
}

std::vector<std::string> cfg_effective_sources(const persisted_config &c_in) {
  persisted_config c = c_in;
  cfg_normalize(c);

  std::vector<std::string> out;
  for (auto &s : c.tc358743_order) uniq_push(out, s);
  for (auto &a : c.aux_sources) uniq_push(out, a.dev);

  // Back-compat: if auxSources is empty but legacy v4l2Sources includes extra devices, include them.
  if (c.aux_sources.empty()) {
    for (auto &s : c.v4l2_sources) uniq_push(out, s);
  }

  return out;
}

json filter_registry_json() {
  json j;
  json filters = json::array();
  {
    json f;
    f["id"] = "mono";
    f["name"] = "Monochrome";
    f["params"] = { {"strength", 1.0} };
    filters.push_back(f);
  }
  {
    json f;
    f["id"] = "sobel";
    f["name"] = "Sobel";
    f["params"] = { {"mode","edgesOnly"}, {"threshold",64}, {"alpha",1.0}, {"invert",false} };
    filters.push_back(f);
  }
  {
    nlohmann::json f;
    f["id"] = "denoise";
    f["name"] = "Denoise";
    f["params"] = { {"radius", 1}, {"strength", 1.0} };
    filters.push_back(f);
  }
  {
    json f;
    f["id"] = "rgbMap";
    f["name"] = "RGB range → RGB value";
    // Use -1/-1 as wildcard for a channel.
    f["params"] = {
      {"rMin", 0},   {"rMax", 127}, {"rOut", 255},
      {"gMin", -1},  {"gMax", -1},  {"gOut", 255},
      {"bMin", -1},  {"bMax", -1},  {"bOut", 127},
      {"aMin", -1},  {"aMax", -1},  {"aOut", 255}
    };
    filters.push_back(f);
  }
  {
    json f;
    f["id"] = "rgbKeyAlpha";
    f["name"] = "RGB key → alpha (black/white mask)";
    f["params"] = {
      {"mode", "black"},        // "black" | "white"
      {"threshold", 16},        // 0..255
      {"keyAlpha", 0},          // 0..255
      {"keepAlpha", 255},       // 0..255
      {"setRgb", false},        // true => force RGB to outRgb
      {"outRgb", "255,255,255"} // CSV string for convenience
    };
    filters.push_back(f);
  }
  {
    json f;
    f["id"] = "m3liteEdgeMask";
    f["name"] = "M3Lite edge mask (combined)";
    f["params"] = json::object(); // no params for now
    filters.push_back(f);
  }
  j["filters"] = filters;
  return j;
}

std::string config_to_json(const persisted_config &c_in) {
  persisted_config c = c_in;
  cfg_normalize(c);

  json j;
  j["v4l2Dev"] = c.v4l2_dev;
  j["v4l2Sources"] = c.v4l2_sources; // legacy display list (still used by your UI)
  j["tc358743Order"] = c.tc358743_order;

  json aux = json::array();
  for (const auto &a : c.aux_sources) {
    json o;
    o["dev"] = a.dev;
    o["mode"] = a.mode;
    o["width"] = a.width;
    o["height"] = a.height;
    o["pixfmt"] = a.pixfmt;
    aux.push_back(o);
  }
  j["auxSources"] = aux;

  j["edidPath"] = c.edid_path;
  j["modesetMatchInput"] = c.do_modeset;
  j["threads"] = c.threads;
  j["bgr"] = c.input_is_bgr;
  j["present"] = c.present_policy;
  j["preferOutputMode"] = c.prefer_output_mode;
  j["preferOutputRes"] = std::to_string(c.prefer_out_w) + "x" + std::to_string(c.prefer_out_h);
  j["viewport"] = c.viewport.set ? (std::to_string(c.viewport.w) + "x" + std::to_string(c.viewport.h)) : "";
  j["listenAddr"] = c.listen_addr;

  // ---- OAK config ----
  j["oakEnable"] = c.oak_enable;
  j["oakMxid"] = c.oak_mxid;
  j["oakW"] = c.oak_w;
  j["oakH"] = c.oak_h;

  json layers = json::array();
  for (const auto &L : c.layers) {
    json jl;
    jl["name"] = L.name;
    jl["type"] = (L.type==LAYER_CROSSHAIR) ? "crosshair" : (L.type==LAYER_GRAPHICS ? "graphics" : "video");
    jl["enabled"] = L.enabled;
    jl["opacity"] = std::clamp(L.opacity, 0.0f, 1.0f);
    jl["invertRel"] = (L.invert_rel==INV_UPPER) ? "upper" : (L.invert_rel==INV_LOWER ? "lower" : "none");
    jl["srcRect"] = std::to_string(L.src_rect.x) + "," + std::to_string(L.src_rect.y) + "," + std::to_string(L.src_rect.w) + "," + std::to_string(L.src_rect.h);
    jl["dstPos"] = std::to_string(L.dst_pos.x) + "," + std::to_string(L.dst_pos.y);
    {
      std::ostringstream sc;
      sc << L.scale_x << "," << L.scale_y;
      jl["scale"] = sc.str();
    }
    if (L.type == LAYER_VIDEO) jl["videoSource"] = L.video_source;

    // ---- OAK per-layer ----
    jl["oakEnabled"] = (L.type == LAYER_VIDEO) ? (bool)L.oak.enabled : false;
    jl["oakRole"] = (L.type == LAYER_VIDEO) ? std::string(oak_role_to_string(L.oak.role)) : std::string("none");

    json fl = json::array();
    if (L.type == LAYER_VIDEO) {
      for (const auto &F : L.filters) {
        json jf;
        if (F.id == FILTER_MONO) {
          jf["id"] = "mono";
          jf["params"] = { {"strength", std::clamp(F.mono_strength, 0.0f, 1.0f)} };
        } else if (F.id == FILTER_SOBEL) {
          jf["id"] = "sobel";
          jf["params"] = {
            {"mode", (F.sobel_mode == filter_cfg::SOBEL_MAGNITUDE) ? "magnitude" : "edgesOnly"},
            {"threshold", (int)F.sobel_threshold},
            {"alpha", std::clamp(F.sobel_alpha, 0.0f, 1.0f)},
            {"invert", F.sobel_invert}
          };
        } else if (F.id == FILTER_DENOISE) {
          jf["id"] = "denoise";
          jf["params"] = {
            {"radius", F.denoise_radius},
            {"strength", std::clamp(F.denoise_strength, 0.0f, 1.0f)}
          };
        } else if (F.id == FILTER_RGB_MAP) {
          jf["id"] = "rgbMap";
          jf["params"] = {
            {"rMin", F.r_min}, {"rMax", F.r_max}, {"rOut", (int)F.r_out},
            {"gMin", F.g_min}, {"gMax", F.g_max}, {"gOut", (int)F.g_out},
            {"bMin", F.b_min}, {"bMax", F.b_max}, {"bOut", (int)F.b_out},
            {"aMin", F.a_min}, {"aMax", F.a_max}, {"aOut", (int)F.a_out},
          };
        } else if (F.id == FILTER_RGB_KEY_ALPHA) {
          jf["id"] = "rgbKeyAlpha";
          jf["params"] = {
            {"mode", (F.key_mode == filter_cfg::KEY_WHITE) ? "white" : "black"},
            {"threshold", F.key_threshold},
            {"keyAlpha", (int)F.key_alpha},
            {"keepAlpha", (int)F.keep_alpha},
            {"setRgb", F.set_rgb},
            {"outRgb", std::to_string((int)F.out_r) + "," + std::to_string((int)F.out_g) + "," + std::to_string((int)F.out_b)}
          };
        } else if (F.id == FILTER_M3LITE_EDGE_MASK) {
          jf["id"] = "m3liteEdgeMask";
          jf["params"] = json::object();
        } else {
          jf["id"] = "unknown";
          jf["params"] = json::object();
        }
        fl.push_back(jf);
      }
    }
    jl["filters"] = fl;

    json xh;
    xh["enabled"] = L.xh.enabled;
    xh["diam"] = std::to_string(L.xh.diam_w) + "x" + std::to_string(L.xh.diam_h);
    xh["center"] = L.xh.center_set ? (std::to_string(L.xh.cx) + "," + std::to_string(L.xh.cy)) : "";
    xh["thickness"] = (int)L.xh.thickness;
    xh["mode"] = L.xh.solid ? "solid" : "invert";
    xh["color"] = std::to_string((int)L.xh.r) + "," + std::to_string((int)L.xh.g) + "," + std::to_string((int)L.xh.b);
    xh["opacity"] = std::clamp(L.xh.opacity, 0.0f, 1.0f);
    xh["invertRel"] = (L.xh.invert_rel==INV_UPPER) ? "upper" : (L.xh.invert_rel==INV_LOWER ? "lower" : "none");
    jl["crosshair"] = xh;

    layers.push_back(jl);
  }
  j["layers"] = layers;

  return j.dump();
}

bool config_from_json_text(const std::string &text, persisted_config &c) {
  json j = json::parse(text, nullptr, false);
  if (j.is_discarded() || !j.is_object()) return false;

  auto get_str = [&](const char *k, std::string &out) {
    if (j.contains(k) && j[k].is_string()) out = j[k].get<std::string>();
  };
  auto get_bool = [&](const char *k, bool &out) {
    if (j.contains(k) && j[k].is_boolean()) out = j[k].get<bool>();
  };
  auto get_int = [&](const char *k, int &out) {
    if (j.contains(k) && j[k].is_number_integer()) out = j[k].get<int>();
  };

  get_str("v4l2Dev", c.v4l2_dev);
  get_str("edidPath", c.edid_path);
  get_bool("modesetMatchInput", c.do_modeset);
  get_int("threads", c.threads);
  get_bool("bgr", c.input_is_bgr);

  std::string present;
  get_str("present", present);
  if (!present.empty() && present_policy_valid(present)) c.present_policy = present;

  get_bool("preferOutputMode", c.prefer_output_mode);

  std::string preferRes;
  get_str("preferOutputRes", preferRes);
  if (!preferRes.empty()) {
    uint32_t w=0,h=0;
    if (parse_dim(preferRes.c_str(), &w,&h)) { c.prefer_out_w=w; c.prefer_out_h=h; }
  }

  std::string viewport;
  get_str("viewport", viewport);
  if (viewport.empty()) c.viewport.set=false;
  else {
    uint32_t w=0,h=0;
    if (parse_dim(viewport.c_str(), &w,&h)) { c.viewport.set=true; c.viewport.w=w; c.viewport.h=h; }
  }

  get_str("listenAddr", c.listen_addr);

  // ---- OAK global config (optional) ----
  if (j.contains("oakEnable") && j["oakEnable"].is_boolean()) c.oak_enable = j["oakEnable"].get<bool>();
  if (j.contains("oakMxid") && j["oakMxid"].is_string()) c.oak_mxid = j["oakMxid"].get<std::string>();
  if (j.contains("oakW") && j["oakW"].is_number_integer()) {
    int w = j["oakW"].get<int>();
    if (w > 0) c.oak_w = (uint32_t)w;
  }
  if (j.contains("oakH") && j["oakH"].is_number_integer()) {
    int h = j["oakH"].get<int>();
    if (h > 0) c.oak_h = (uint32_t)h;
  }

  // New: tc358743Order
  if (j.contains("tc358743Order") && j["tc358743Order"].is_array()) {
    c.tc358743_order.clear();
    for (const auto &x : j["tc358743Order"]) if (x.is_string()) c.tc358743_order.push_back(x.get<std::string>());
  }

  // New: auxSources
  if (j.contains("auxSources") && j["auxSources"].is_array()) {
    c.aux_sources.clear();
    for (const auto &x : j["auxSources"]) {
      if (!x.is_object()) continue;
      aux_source_cfg a;
      if (x.contains("dev") && x["dev"].is_string()) a.dev = x["dev"].get<std::string>();
      if (x.contains("mode") && x["mode"].is_string()) a.mode = x["mode"].get<std::string>();
      if (x.contains("width") && x["width"].is_number_integer()) a.width = (uint32_t)x["width"].get<int>();
      if (x.contains("height") && x["height"].is_number_integer()) a.height = (uint32_t)x["height"].get<int>();
      if (x.contains("pixfmt") && x["pixfmt"].is_string()) a.pixfmt = x["pixfmt"].get<std::string>();
      c.aux_sources.push_back(a);
    }
  }

  // Legacy: v4l2Sources (string array)
  if (j.contains("v4l2Sources") && j["v4l2Sources"].is_array()) {
    c.v4l2_sources.clear();
    for (const auto &x : j["v4l2Sources"]) if (x.is_string()) c.v4l2_sources.push_back(x.get<std::string>());
  }

  // Layers
  if (j.contains("layers") && j["layers"].is_array()) {
    c.layers.clear();
    for (const auto &jl : j["layers"]) {
      if (!jl.is_object()) continue;
      layer_cfg L;

      if (jl.contains("name") && jl["name"].is_string()) L.name = jl["name"].get<std::string>();
      std::string type = jl.value("type", "video");
      if (type=="crosshair") L.type = LAYER_CROSSHAIR;
      else if (type=="graphics") L.type = LAYER_GRAPHICS;
      else L.type = LAYER_VIDEO;

      if (L.type == LAYER_VIDEO && jl.contains("videoSource") && jl["videoSource"].is_string()) {
        L.video_source = jl["videoSource"].get<std::string>();
      }

      L.enabled = jl.value("enabled", true);
      L.opacity = (float)std::clamp(jl.value("opacity", 1.0), 0.0, 1.0);

      std::string inv = jl.value("invertRel", "none");
      if (inv=="lower") L.invert_rel = INV_LOWER;
      else if (inv=="upper") L.invert_rel = INV_UPPER;
      else L.invert_rel = INV_NONE;

      std::string srcRect = jl.value("srcRect", "0,0,0,0");
      {
        unsigned long x=0,y=0,w=0,h=0;
        if (sscanf(srcRect.c_str(), "%lu,%lu,%lu,%lu", &x,&y,&w,&h)==4 && w>0 && h>0) {
          L.src_rect.x=(uint32_t)x; L.src_rect.y=(uint32_t)y; L.src_rect.w=(uint32_t)w; L.src_rect.h=(uint32_t)h;
        }
      }

      std::string dstPos = jl.value("dstPos", "0,0");
      {
        int x=0,y=0;
        if (sscanf(dstPos.c_str(), "%d,%d", &x,&y)==2) { L.dst_pos.x=x; L.dst_pos.y=y; }
      }

      std::string scale = jl.value("scale", "1.0,1.0");
      {
        double sx=1.0, sy=1.0;
        if (sscanf(scale.c_str(), "%lf,%lf", &sx,&sy)==2 && sx>0.0001 && sy>0.0001) {
          L.scale_x=(float)sx; L.scale_y=(float)sy;
        }
      }

      // ---- OAK per-layer ----
      // Defaults already: enabled=false, role=none
      if (L.type == LAYER_VIDEO) {
        if (jl.contains("oakEnabled") && jl["oakEnabled"].is_boolean()) L.oak.enabled = jl["oakEnabled"].get<bool>();
        if (jl.contains("oakRole") && jl["oakRole"].is_string()) L.oak.role = oak_role_from_string(jl["oakRole"].get<std::string>());
      } else {
        L.oak.enabled = false;
        L.oak.role = OAK_NONE;
      }

      // Filters
      if (jl.contains("filters") && jl["filters"].is_array() && L.type == LAYER_VIDEO) {
        for (const auto &jf : jl["filters"]) {
          if (!jf.is_object()) continue;
          std::string id = jf.value("id", "");
          if (id.empty()) continue;
          filter_cfg fc;
          if (id == "mono") {
            fc.id = FILTER_MONO;
            if (jf.contains("params") && jf["params"].is_object()) {
              double st = jf["params"].value("strength", 1.0);
              fc.mono_strength = (float)std::clamp(st, 0.0, 1.0);
            }
          } else if (id == "sobel") {
            fc.id = FILTER_SOBEL;
            if (jf.contains("params") && jf["params"].is_object()) {
              std::string mode = jf["params"].value("mode", "edgesOnly");
              fc.sobel_mode = (mode == "magnitude") ? filter_cfg::SOBEL_MAGNITUDE : filter_cfg::SOBEL_EDGES_ONLY;
              int th = jf["params"].value("threshold", 64);
              if (th < 0) th = 0;
              if (th > 255) th = 255;
              fc.sobel_threshold = (uint8_t)th;
              double a = jf["params"].value("alpha", 1.0);
              fc.sobel_alpha = (float)std::clamp(a, 0.0, 1.0);
              fc.sobel_invert = jf["params"].value("invert", false);
            }
          } else if (id == "denoise") {
            fc.id = FILTER_DENOISE;
            if (jf.contains("params") && jf["params"].is_object()) {
              int r = jf["params"].value("radius", 1);
              if (r < 1) r = 1;
              if (r > 8) r = 8;
              fc.denoise_radius = r;

              double st = jf["params"].value("strength", 1.0);
              fc.denoise_strength = (float)std::clamp(st, 0.0, 1.0);
            }
          } else if (id == "rgbMap") {
            fc.id = FILTER_RGB_MAP;
            if (jf.contains("params") && jf["params"].is_object()) {
              const auto &p = jf["params"];

              auto get_i = [&](const char *k, int defv) -> int {
                if (p.contains(k) && p[k].is_number_integer()) return p[k].get<int>();
                return defv;
              };
              auto get_u8 = [&](const char *k, int defv) -> uint8_t {
                int v = defv;
                if (p.contains(k) && p[k].is_number_integer()) v = p[k].get<int>();
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                return (uint8_t)v;
              };
              auto clamp_minmax = [&](int &mn, int &mx) {
                // wildcard if both -1
                if (mn == -1 && mx == -1) return;
                // allow user to specify just one side? treat missing as wildcard is tricky;
                // here: clamp to [0..255] and fix ordering
                mn = std::max(0, std::min(255, mn));
                mx = std::max(0, std::min(255, mx));
                if (mx < mn) std::swap(mx, mn);
              };

              fc.r_min = get_i("rMin", 0);
              fc.r_max = get_i("rMax", 255);
              fc.r_out = get_u8("rOut", 255);

              fc.g_min = get_i("gMin", -1);
              fc.g_max = get_i("gMax", -1);
              fc.g_out = get_u8("gOut", 255);

              fc.b_min = get_i("bMin", -1);
              fc.b_max = get_i("bMax", -1);
              fc.b_out = get_u8("bOut", 255);

              fc.a_min = get_i("aMin", -1);
              fc.a_max = get_i("aMax", -1);
              fc.a_out = get_u8("aOut", 255);

              clamp_minmax(fc.r_min, fc.r_max);
              clamp_minmax(fc.g_min, fc.g_max);
              clamp_minmax(fc.b_min, fc.b_max);
              clamp_minmax(fc.a_min, fc.a_max);
            }
          } else if (id == "rgbKeyAlpha") {
            fc.id = FILTER_RGB_KEY_ALPHA;
            if (jf.contains("params") && jf["params"].is_object()) {
              const auto &p = jf["params"];

              std::string mode = p.value("mode", "black");
              fc.key_mode = (mode == "white") ? filter_cfg::KEY_WHITE : filter_cfg::KEY_BLACK;

              int th = p.value("threshold", 16);
              if (th < 0) th = 0;
              if (th > 255) th = 255;
              fc.key_threshold = th;

              int ka = p.value("keyAlpha", 0);
              if (ka < 0) ka = 0;
              if (ka > 255) ka = 255;
              fc.key_alpha = (uint8_t)ka;

              int na = p.value("keepAlpha", 255);
              if (na < 0) na = 0;
              if (na > 255) na = 255;
              fc.keep_alpha = (uint8_t)na;

              fc.set_rgb = p.value("setRgb", false);

              std::string outRgb = p.value("outRgb", "255,255,255");
              uint8_t rr=255, gg=255, bb=255;
              if (parse_rgb_csv(outRgb.c_str(), &rr, &gg, &bb)) {
                fc.out_r = rr; fc.out_g = gg; fc.out_b = bb;
              }
            }
          } else if (id == "m3liteEdgeMask") {
            fc.id = FILTER_M3LITE_EDGE_MASK;
            // no params
          } else {
            continue;
          }
          L.filters.push_back(fc);
        }
      }

      // Crosshair
      if (jl.contains("crosshair") && jl["crosshair"].is_object()) {
        const auto &xh = jl["crosshair"];
        L.xh.enabled = xh.value("enabled", false);
        std::string diam = xh.value("diam", "50x50");
        { uint32_t w=0,h=0; if (parse_dim(diam.c_str(), &w,&h)) { L.xh.diam_w=w; L.xh.diam_h=h; } }
        std::string center = xh.value("center", "");
        if (center.empty()) L.xh.center_set=false;
        else {
          int32_t cx=0,cy=0;
          if (parse_point_csv(center.c_str(), &cx,&cy)) { L.xh.center_set=true; L.xh.cx=cx; L.xh.cy=cy; }
        }
        int th = xh.value("thickness", 1);
        if (th < 1) th = 1;
        if (th > 99) th = 99;
        L.xh.thickness = (uint32_t)th;
        std::string mode = xh.value("mode", "solid");
        L.xh.solid = (mode != "invert");
        std::string color = xh.value("color", "255,255,255");
        { uint8_t r=255,g=255,b=255; if (parse_rgb_csv(color.c_str(), &r,&g,&b)) { L.xh.r=r; L.xh.g=g; L.xh.b=b; } }
        L.xh.opacity = (float)std::clamp(xh.value("opacity", 1.0), 0.0, 1.0);
        std::string xin = xh.value("invertRel", "none");
        if (xin=="lower") L.xh.invert_rel = INV_LOWER;
        else if (xin=="upper") L.xh.invert_rel = INV_UPPER;
        else L.xh.invert_rel = INV_NONE;
      }

      c.layers.push_back(L);
    }
  }

  cfg_normalize(c);
  return true;
}

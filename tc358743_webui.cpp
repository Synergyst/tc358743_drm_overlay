#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include <cstdio>

#include "third_party/httplib.h"
#include "tc358743_webui.h"

static std::mutex g_mtx;

static std::string (*g_cfg_json)() = nullptr;
static bool (*g_apply)(const std::string&, std::string&, int&) = nullptr;
static std::string (*g_status)() = nullptr;
static std::string (*g_filter_defs)() = nullptr;

// NEW: V4L2 caps provider
static std::string (*g_v4l2_caps)(const std::string &dev) = nullptr;

static std::atomic<bool> *g_quit = nullptr;
static std::vector<uint8_t> g_ref_png;
static std::string g_listen_addr = "0.0.0.0";

void webui_set_config_json_provider(std::string (*fn)()) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_cfg_json = fn;
}
void webui_set_apply_handler(bool (*fn)(const std::string&, std::string&, int&)) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_apply = fn;
}
void webui_set_status_provider(std::string (*fn)()) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_status = fn;
}
void webui_set_quit_flag(std::atomic<bool> *quit_flag) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_quit = quit_flag;
}
void webui_set_reference_png(const std::vector<uint8_t> &png_bytes) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_ref_png = png_bytes;
}
void webui_set_listen_address(const std::string &addr) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_listen_addr = addr;
}
void webui_set_filter_defs_provider(std::string (*fn)()) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_filter_defs = fn;
}
// Backward-compatible alias for older code.
void webui_set_filters_provider(std::string (*fn)()) {
  webui_set_filter_defs_provider(fn);
}

// NEW: V4L2 caps provider setter
void webui_set_v4l2_caps_provider(std::string (*fn)(const std::string &dev)) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_v4l2_caps = fn;
}

/*
  Notes re: per-layer filter stacks (video layers only for now):
  - UI stores filters as: layer.filters = [{id, enabled, params}, ...]
  - Filters are intended to apply in *layer output space* (post-scale) backend-side.
  - This WebUI supports dynamic enumeration via GET /api/filters (if backend implements it),
    but also has a built-in fallback list so you can land UI first.
*/
static const char *kIndexHtml = R"HTML(<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>Overlay Editor</title>
  <style>
    :root{
      --bg:#0b0f14; --panel:#121826; --panel2:#0f1626; --muted:#7d8aa6; --fg:#e8eefc;
      --accent:#58a6ff; --line:#243047; --warn:#ffd166; --bad:#ff5d5d; --good:#5bff9c;
      --shadow: rgba(0,0,0,0.35);
      --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    * { box-sizing:border-box; }
    body { margin:0; font-family:system-ui, sans-serif; background:var(--bg); color:var(--fg); }
    header { padding:12px 14px; border-bottom:1px solid var(--line); display:flex; align-items:center; gap:12px; position:sticky; top:0; background:rgba(11,15,20,0.88); backdrop-filter: blur(8px); z-index:5; }
    header h1 { font-size:15px; margin:0; font-weight:700; letter-spacing:0.2px; }
    header .sp { flex:1; }
    header button { background:var(--accent); color:#08121f; border:0; padding:10px 12px; border-radius:10px; font-weight:800; cursor:pointer; }
    header button.secondary { background:#1b2436; color:var(--fg); border:1px solid var(--line); font-weight:700; }
    header button.danger { background:#2a1212; border:1px solid #5d2a2a; color:#ffd6d6; }
    .wrap { display:grid; grid-template-columns: 1.35fr 0.95fr; gap:14px; padding:14px; align-items:start; }
    @media (max-width: 1100px) { .wrap { grid-template-columns: 1fr; } }
    .panel { background:var(--panel); border:1px solid var(--line); border-radius:14px; overflow:hidden; box-shadow: 0 6px 28px var(--shadow); }
    .panel .hd { padding:10px 12px; border-bottom:1px solid var(--line); display:flex; align-items:center; justify-content:space-between; gap:12px; }
    .panel .bd { padding:12px; }
    .title { font-weight:850; }
    .muted { color:var(--muted); font-size:12px; line-height:1.35; }
    .mono { font-family:var(--mono); }
    .status { font-family:var(--mono); font-size:12px; white-space:pre; color:#cfd8ee; background:var(--panel2); border:1px solid var(--line); border-radius:12px; padding:10px; overflow:auto; max-height:220px; }
    .grid2 { display:grid; grid-template-columns: 1fr 1fr; gap:10px; }
    @media (max-width: 700px){ .grid2 { grid-template-columns: 1fr; } }
    label { display:block; font-size:12px; color:var(--muted); margin-bottom:4px; }
    input, select, textarea {
      width:100%;
      padding:10px 10px;
      border-radius:10px;
      background:#0b1020;
      color:var(--fg);
      border:1px solid var(--line);
      outline:none;
    }
    input:focus, select:focus, textarea:focus { border-color: rgba(88,166,255,0.65); box-shadow: 0 0 0 3px rgba(88,166,255,0.12); }
    .rowBtns { display:flex; gap:10px; flex-wrap:wrap; align-items:center; }
    .rowBtns button { background:#1b2436; color:var(--fg); border:1px solid var(--line); padding:10px 12px; border-radius:10px; cursor:pointer; font-weight:700; }
    .rowBtns button.primary { background:var(--accent); color:#08121f; border:0; font-weight:900; }
    .rowBtns button.warn { background: #2a2413; border:1px solid #56451a; color:#ffe7a6; }
    .rowBtns button.danger { background:#2a1212; border:1px solid #5d2a2a; color:#ffd6d6; }
    .rowBtns .pill { padding:6px 10px; border:1px solid var(--line); border-radius:999px; font-size:12px; color: var(--muted); background: rgba(0,0,0,0.15); }
    table { width:100%; border-collapse:collapse; font-size:12px; }
    th,td { border-top:1px solid var(--line); padding:8px; vertical-align:top; }
    th { text-align:left; color:var(--muted); font-weight:800; }
    .right { text-align:right; }
    .card { background: var(--panel2); border:1px solid var(--line); border-radius: 12px; padding: 10px; }
    .card .hd2 { display:flex; align-items:center; justify-content:space-between; gap:10px; margin-bottom:8px; }
    .chip { font-size:12px; border:1px solid var(--line); border-radius:999px; padding:4px 8px; color:var(--muted); }
    .err { color:#ffd6d6; background: rgba(255,93,93,0.12); border:1px solid rgba(255,93,93,0.35); border-radius:12px; padding:10px; }
    .help { font-size: 12px; color:var(--muted); line-height: 1.35; }
    .hr { height:1px; background: var(--line); margin: 12px 0; }
    .sectionTitle { font-weight:900; margin:0 0 6px 0; font-size: 13px; }
    .kbd { font-family: var(--mono); border:1px solid var(--line); background: rgba(0,0,0,0.2); border-radius: 6px; padding: 2px 6px; font-size: 12px; color: var(--muted); }
    /* Viewport */
    .viewportWrap { position:relative; background:#05080d; border-radius:12px; overflow:hidden; border:1px solid var(--line); }
    .viewportWrap .checker { position:absolute; inset:0; background:
      linear-gradient(45deg, rgba(255,255,255,0.06) 25%, transparent 25%),
      linear-gradient(-45deg, rgba(255,255,255,0.06) 25%, transparent 25%),
      linear-gradient(45deg, transparent 75%, rgba(255,255,255,0.06) 75%),
      linear-gradient(-45deg, transparent 75%, rgba(255,255,255,0.06) 75%);
      background-size:24px 24px;
      background-position:0 0, 0 12px, 12px -12px, -12px 0px;
      opacity:0.35;
      pointer-events:none;
    }
    #refImg { width:100%; display:block; image-rendering: pixelated; user-select:none; -webkit-user-drag:none; }
    #overlayStage { position:absolute; inset:0; touch-action:none; }
    #xhPreview { position:absolute; inset:0; pointer-events:none; }
    .box {
      position:absolute;
      border:2px solid rgba(88,166,255,0.95);
      background:rgba(88,166,255,0.10);
      border-radius:10px;
      user-select:none;
      pointer-events:auto;
    }
    .box.selected { border-color: var(--warn); background: rgba(255,209,102,0.11); }
    .box.disabled { opacity:0.35; filter: grayscale(0.2); }
    .box .label {
      position:absolute; left:8px; top:6px;
      font-size:12px; font-weight:900;
      background:rgba(0,0,0,0.55);
      padding:4px 6px; border-radius: 10px;
      pointer-events:none;
    }
    .box .inner {
      position:absolute; inset:0;
      border-radius:10px;
      background: transparent;
      cursor: grab;
    }
    .box.selected .inner { background: rgba(255,209,102,0.06); }
    .box .inner:active { cursor: grabbing; }
    .handle { width:12px; height:12px; background: var(--warn); border-radius:4px; position:absolute;
              box-shadow: 0 0 0 2px rgba(0,0,0,0.45); }
    .h-nw{left:-6px;top:-6px;cursor:nwse-resize;}
    .h-ne{right:-6px;top:-6px;cursor:nesw-resize;}
    .h-sw{left:-6px;bottom:-6px;cursor:nesw-resize;}
    .h-se{right:-6px;bottom:-6px;cursor:nwse-resize;}
    .h-n{left:calc(50% - 6px);top:-6px;cursor:ns-resize;}
    .h-s{left:calc(50% - 6px);bottom:-6px;cursor:ns-resize;}
    .h-w{left:-6px;top:calc(50% - 6px);cursor:ew-resize;}
    .h-e{right:-6px;top:calc(50% - 6px);cursor:ew-resize;}
    /* In-layer crop UI */
    .cropRect {
      position:absolute;
      border:2px dashed rgba(255,209,102,0.98);
      background: rgba(255,209,102,0.14);
      border-radius: 10px;
      pointer-events:auto;
      cursor: crosshair;
    }
    .cropRect .ch { width:12px; height:12px; background: rgba(255,209,102,0.95); border-radius:4px; position:absolute;
                    box-shadow: 0 0 0 2px rgba(0,0,0,0.45); }
    .cropRect .c-nw{left:-6px;top:-6px;cursor:nwse-resize;}
    .cropRect .c-ne{right:-6px;top:-6px;cursor:nesw-resize;}
    .cropRect .c-sw{left:-6px;bottom:-6px;cursor:nesw-resize;}
    .cropRect .c-se{right:-6px;bottom:-6px;cursor:nwse-resize;}
    .cropRect .cap {
      position:absolute; left:8px; top:6px;
      font-size:12px; font-weight:900;
      background:rgba(0,0,0,0.55);
      padding:4px 6px; border-radius: 10px;
      pointer-events:none;
    }
    /* Filters UI */
    .fRow { display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
    .fRow .mini { padding:8px 10px; border-radius:10px; background:#121a2b; border:1px solid var(--line); color:var(--fg); font-size:12px; }
    .fStack { display:flex; flex-direction:column; gap:10px; }
    .fItem { border:1px solid var(--line); border-radius:12px; padding:10px; background: rgba(0,0,0,0.16); }
    .fItem .top { display:flex; align-items:center; justify-content:space-between; gap:10px; }
    .fItem .top .left { display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
    .fItem .top .name { font-weight:900; }
    .fItem .params { margin-top:10px; display:grid; grid-template-columns: 1fr 1fr; gap:10px; }
    @media (max-width: 700px){ .fItem .params { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
<header>
  <h1>Overlay Editor</h1>
  <span class="pill mono" id="pillMode">mode: select</span>
  <div class="sp"></div>
  <button class="secondary" id="btnReload">Reload</button>
  <button class="primary" id="btnApply">Apply</button>
</header>
<div class="wrap">
  <div class="panel">
    <div class="hd">
      <div>
        <div class="title">Viewport</div>
        <div class="muted">
          Drag in the inner highlighted area to move. Drag handles to resize.
          <span class="mono">(Shift snaps to 10px)</span>
        </div>
      </div>
      <div class="rowBtns">
        <button id="btnAddVideo">Add Video</button>
        <button id="btnAddCrosshair">Add Crosshair</button>
        <button class="warn" id="btnCrop">Crop</button>
      </div>
    </div>
    <div class="bd">
      <div class="viewportWrap" id="vpWrap">
        <div class="checker"></div>
        <img id="refImg" src="/ref.png" />
        <svg id="xhPreview"></svg>
        <div id="overlayStage"></div>
      </div>
      <div id="cropPanel" class="card" style="display:none; margin-top:10px;">
        <div class="hd2">
          <div class="title">Crop mode (inside selected video layer)</div>
          <div class="chip">Drag inside the layer to draw crop rect</div>
        </div>
        <div class="help">
          Keys: <span class="kbd">Esc</span> cancel, <span class="kbd">Enter</span> commit.
        </div>
        <div class="hr"></div>
        <div class="grid2">
          <div>
            <label>Crop action</label>
            <select id="cropMode">
              <option value="keep">Keep (crop to this area)</option>
              <option value="removeSplit">Remove (split remainder into TR/BL/BR)</option>
            </select>
          </div>
          <div>
            <label>Crop readout</label>
            <div class="help mono" id="cropReadout">(none)</div>
            <div class="rowBtns" style="margin-top:10px;">
              <button class="secondary" id="btnCropCancel">Cancel</button>
              <button class="primary" id="btnCropCommit">Commit</button>
            </div>
          </div>
        </div>
      </div>
      <div style="margin-top:10px;" class="muted">
        Reference frame captured once at startup. Preview is not live video.
      </div>
    </div>
  </div>
  <div class="panel">
    <div class="hd">
      <div>
        <div class="title">Config & Inspector</div>
        <div class="muted">Use Apply to persist changes. Status updates every second.</div>
      </div>
      <div class="rowBtns">
        <button class="danger" id="btnQuit">Quit</button>
      </div>
    </div>
    <div class="bd">
      <div class="status" id="status">Loading...</div>
      <div style="height:10px;"></div>
      <div class="card">
        <div class="hd2">
          <div class="title">Global</div>
          <span class="chip">viewport + mapping</span>
        </div>
        <div class="grid2">
          <div>
            <label>V4L2 Device</label>
            <input id="v4l2Dev" type="text" placeholder="/dev/v4l/by-path/platform-fe800000.csi-video-index0"/>
            <div class="help">Default source used by new video layers (unless layer overrides videoSource).</div>
          </div>
          <div>
            <label>V4L2 Sources (CSV)</label>
            <input id="v4l2Sources" class="mono" type="text" placeholder="/dev/v4l/by-path/platform-fe800000.csi-video-index0,/dev/v4l/by-path/platform-fe801000.csi-video-index0"/>
            <div class="help">Optional list of sources; used for per-layer selection and status display.</div>
          </div>
          <div>
            <label>EDID Path</label>
            <input id="edidPath" type="text" placeholder=""/>
          </div>
          <div>
            <label>Present Policy</label>
            <select id="present">
              <option value="stretch">stretch</option>
              <option value="fit">fit</option>
              <option value="1:1">1:1</option>
            </select>
          </div>
          <div>
            <label>Viewport (WxH) blank = input</label>
            <input id="viewport" type="text" placeholder="1920x1080"/>
            <div class="help" id="vpHelp"></div>
          </div>
          <div>
            <label><input id="preferOutputMode" type="checkbox"/> Prefer output mode</label>
          </div>
          <div>
            <label>Prefer Output Res (WxH)</label>
            <input id="preferOutputRes" type="text" placeholder="1280x720"/>
          </div>
          <div>
            <label>Threads</label>
            <input id="threads" type="number" min="1" max="64" step="1"/>
            <div class="help">0 = auto</div>
          </div>
          <div>
            <label>Input is BGR</label>
            <input id="bgr" type="checkbox"/>
          </div>
        </div>
      </div>
      <div style="height:12px;"></div>
      <div class="card">
        <div class="hd2">
          <div class="title">Layers</div>
          <span class="chip" id="layerCount">0</span>
        </div>
        <table>
          <thead>
            <tr>
              <th class="right">#</th>
              <th>Name</th>
              <th>Type</th>
              <th>Enabled</th>
              <th>Opacity</th>
              <th>Invert</th>
              <th class="right">Actions</th>
            </tr>
          </thead>
          <tbody id="layersTable"></tbody>
        </table>
      </div>
      <div style="height:12px;"></div>
      <div class="card" id="inspectorCard">
        <div class="hd2">
          <div class="title">Inspector</div>
          <span class="chip" id="inspectorChip">no selection</span>
        </div>
        <div class="help" id="inspectorHelp">
          Select a layer by clicking it in the viewport or in the table.
        </div>
        <div class="hr"></div>
        <div id="inspectorBody" style="display:none;">
          <div class="grid2">
            <div>
              <label>Name</label>
              <input id="iName" type="text"/>
            </div>
            <div>
              <label>Type</label>
              <select id="iType" disabled>
                <option value="video">video</option>
                <option value="crosshair">crosshair</option>
                <option value="graphics">graphics</option>
              </select>
            </div>
            <div>
              <label><input id="iEnabled" type="checkbox"/> Enabled</label>
            </div>
            <div>
              <label>Opacity (0..1)</label>
              <input id="iOpacity" type="number" min="0" max="1" step="0.01"/>
            </div>
            <div>
              <label>InvertRel</label>
              <select id="iInvertRel">
                <option value="none">none</option>
                <option value="lower">lower</option>
                <option value="upper">upper</option>
              </select>
            </div>
            <div>
              <label>Quick actions</label>
              <div class="rowBtns">
                <button id="btnDupLayer">Duplicate</button>
              </div>
            </div>
          </div>
          <div class="hr"></div>
          <div id="videoInspector" style="display:none;">
            <div class="sectionTitle">Video layer</div>
            <div class="grid2">
              <div>
                <label>Source (videoSource)</label>
                <select id="iVideoSource"></select>
                <div class="help" id="iVideoSourceHelp">Auto-disables at runtime if selected source is not active.</div>
              </div>
              <div>
                <label>srcRect (x,y,w,h)</label>
                <input id="iSrcRect" class="mono" type="text" placeholder="0,0,640,360"/>
                <div class="help" id="iSrcRectHelp"></div>
              </div>
              <div>
                <label>dstPos (x,y)</label>
                <input id="iDstPos" class="mono" type="text" placeholder="0,0"/>
                <div class="help">Updates live when you drag the layer.</div>
              </div>
              <div>
                <label>scale (sx,sy)</label>
                <input id="iScale" class="mono" type="text" placeholder="1.0,1.0"/>
                <div class="help">Updates live when you resize the layer.</div>
              </div>
              <div>
                <label>Clamp srcRect to capture</label>
                <button id="btnClampSrc" class="secondary">Clamp srcRect now</button>
                <div class="help">Clamps to reference image size as proxy for capture size.</div>
              </div>
            </div>
            <div class="hr"></div>
            <div class="sectionTitle">Filters (video only)</div>
            <div class="help">
              Filters are applied per-video-layer in post-scale layer pixel space (backend-side).
              Stack order: top-to-bottom.
            </div>
            <div style="height:8px;"></div>
            <div class="fRow">
              <select id="fAddType"></select>
              <button id="btnAddFilter" class="secondary">Add filter</button>
              <span class="chip" id="fCountChip">0</span>
            </div>
            <div style="height:10px;"></div>
            <div id="filtersBox" class="fStack"></div>
          </div>
          <div id="crossInspector" style="display:none;">
            <div class="sectionTitle">Crosshair layer</div>
            <div class="grid2">
              <div>
                <label><input id="iXEnabled" type="checkbox"/> Crosshair enabled</label>
              </div>
              <div>
                <label>Crosshair opacity (0..1)</label>
                <input id="iXOpacity" type="number" min="0" max="1" step="0.01"/>
              </div>
              <div>
                <label>Diameter (WxH)</label>
                <input id="iXDiam" class="mono" type="text" placeholder="120x120"/>
              </div>
              <div>
                <label>Thickness (1..99)</label>
                <input id="iXThick" type="number" min="1" max="99" step="1"/>
              </div>
              <div>
                <label>Center mode</label>
                <select id="iXCenterMode">
                  <option value="auto">auto (output center)</option>
                  <option value="manual">manual</option>
                </select>
              </div>
              <div>
                <label>Center (x,y) output px</label>
                <input id="iXCenter" class="mono" type="text" placeholder=""/>
              </div>
              <div>
                <label>Mode</label>
                <select id="iXMode">
                  <option value="solid">solid</option>
                  <option value="invert">invert</option>
                </select>
              </div>
              <div>
                <label>Color (r,g,b)</label>
                <input id="iXColor" class="mono" type="text" placeholder="255,0,0"/>
              </div>
              <div>
                <label>InvertRel (crosshair)</label>
                <select id="iXInvertRel">
                  <option value="none">none</option>
                  <option value="lower">lower</option>
                  <option value="upper">upper</option>
                </select>
              </div>
            </div>
            <div class="help" style="margin-top:8px;">
              Crosshair preview is shown in the viewport. If backend rendering still doesn't show it on HDMI output,
              that's backend-side, not UI.
            </div>
          </div>
        </div>
      </div>
      <div style="height:12px;"></div>
      <div class="card">
        <div class="hd2">
          <div class="title">Validation</div>
          <span class="chip">pre-apply checks</span>
        </div>
        <div id="validationBox" class="help mono">ok</div>
      </div>
    </div>
  </div>
</div>
<script>
const LIMITS = {
  opacity: {min:0, max:1},
  thickness: {min:1, max:99},
  scale: {min:0.0001, max:64},
  dim: {min:1, max:16384},
};
function clamp(v, lo, hi){ return Math.min(hi, Math.max(lo, v)); }
function isNum(x){ return typeof x === 'number' && isFinite(x); }
function round(v){ return Math.round(v); }
function fmt(v){ return (Math.round(v*10000)/10000).toString(); }
function deepClone(o){ return JSON.parse(JSON.stringify(o)); }
function parseDimWxH(s){
  if (!s) return null;
  const m = String(s).trim().match(/^(\d+)\s*[xX]\s*(\d+)$/);
  if (!m) return null;
  const w = Number(m[1]), h = Number(m[2]);
  if (!isFinite(w) || !isFinite(h)) return null;
  return {w, h};
}
function parseCSVNums(s, n){
  if (!s) return null;
  const parts = String(s).split(',').map(x => x.trim()).filter(x => x.length);
  if (parts.length !== n) return null;
  const vals = parts.map(x => Number(x));
  if (vals.some(v => !isFinite(v))) return null;
  return vals;
}
function parseRect(s){
  const v = parseCSVNums(s, 4);
  if (!v) return null;
  return {x: v[0], y: v[1], w: v[2], h: v[3]};
}
function parsePos(s){
  const v = parseCSVNums(s, 2);
  if (!v) return null;
  return {x: v[0], y: v[1]};
}
function parseScale(s){
  const v = parseCSVNums(s, 2);
  if (!v) return null;
  return {sx: v[0], sy: v[1]};
}
function parseRGB(s){
  const v = parseCSVNums(s, 3);
  if (!v) return null;
  return {r:v[0], g:v[1], b:v[2]};
}
function parseCSVStrings(s){
  if (!s) return [];
  return String(s).split(',').map(x => x.trim()).filter(x => x.length);
}
async function api(path, opts) {
  const r = await fetch(path, opts);
  const t = await r.text();
  let j;
  try { j = JSON.parse(t); } catch { j = { raw: t }; }
  if (!r.ok) throw new Error(j.error || t);
  return j;
}
let cfg = null;
let selectedIdx = -1;
let runtime = null;
let ui = {
  mode: 'select',
  crop: { active:false, rect:null, drag:null }
};
let stage = {
  vpw: 0,
  vph: 0,
  stageW: 0,
  stageH: 0,
  imgNaturalW: 0,
  imgNaturalH: 0,
};
let stageDrag = null;
// Filter registry
let filterDefs = null;

const FILTER_FALLBACK = {
  filters: [
    {
      id: "mono",
      name: "Monochrome",
      params: [
        { k:"strength", type:"float", min:0.0, max:1.0, default:1.0 }
      ]
    },
    {
      id: "sobel",
      name: "Sobel",
      params: [
        { k:"mode", type:"enum", values:["edgesOnly","magnitude"], default:"edgesOnly" },
        { k:"threshold", type:"int", min:0, max:255, default:64 },
        { k:"alpha", type:"float", min:0.0, max:1.0, default:1.0 },
        { k:"invert", type:"bool", default:false }
      ]
    },
    {
      id: "denoise",
      name: "Denoise",
      params: [
        { k:"radius", type:"int", min:1, max:8, default:1 },
        { k:"strength", type:"float", min:0.0, max:8.0, default:1.0 }
      ]
    },
    {
      id: "rgbMap",
      name: "RGB range → RGB value",
      params: [
        { k:"rMin", type:"int", min:-1, max:255, default:0 },
        { k:"rMax", type:"int", min:-1, max:255, default:255 },
        { k:"rOut", type:"int", min:0,  max:255, default:255 },

        { k:"gMin", type:"int", min:-1, max:255, default:-1 },
        { k:"gMax", type:"int", min:-1, max:255, default:-1 },
        { k:"gOut", type:"int", min:0,  max:255, default:255 },

        { k:"bMin", type:"int", min:-1, max:255, default:-1 },
        { k:"bMax", type:"int", min:-1, max:255, default:-1 },
        { k:"bOut", type:"int", min:0,  max:255, default:127 },

        { k:"aMin", type:"int", min:-1, max:255, default:-1 },
        { k:"aMax", type:"int", min:-1, max:255, default:-1 },
        { k:"aOut", type:"int", min:0,  max:255, default:255 }
      ]
    },
    {
      id: "rgbKeyAlpha",
      name: "RGB key → alpha (black/white mask)",
      params: [
        { k:"mode", type:"enum", values:["black","white"], default:"black" },
        { k:"threshold", type:"int", min:0, max:255, default:16 },
        { k:"keyAlpha", type:"int", min:0, max:255, default:0 },
        { k:"keepAlpha", type:"int", min:0, max:255, default:255 },
        { k:"setRgb", type:"bool", default:false },
        { k:"outRgb", type:"string", default:"255,255,255" }
      ]
    }
  ]
};
function schemaForKnownFilterId(id){
  if (id === 'mono') return FILTER_FALLBACK.filters[0].params;
  if (id === 'sobel') return FILTER_FALLBACK.filters[1].params;
  if (id === 'denoise') return FILTER_FALLBACK.filters[2].params;
  if (id === 'rgbMap') return FILTER_FALLBACK.filters[3].params;
  if (id === 'rgbKeyAlpha') return FILTER_FALLBACK.filters[4].params;
  return [];
}
function normalizeFilterDef(def){
  if (!def || typeof def !== 'object') return { id:'', name:'', _params:[] };
  const out = {...def};
  const p = out.params;
  if (Array.isArray(p)) {
    out._params = p;
  } else if (p && typeof p === 'object') {
    out._params = schemaForKnownFilterId(out.id);
  } else {
    out._params = schemaForKnownFilterId(out.id);
  }
  if (!out.id) out.id = '';
  if (!out.name) out.name = out.id;
  return out;
}
function normalizeFilterDefsObj(obj){
  const o = (obj && typeof obj === 'object') ? obj : {};
  const arr = Array.isArray(o.filters) ? o.filters : [];
  return { filters: arr.map(normalizeFilterDef) };
}
function getFilterDefs(){
  const raw = (filterDefs && filterDefs.filters) ? filterDefs : FILTER_FALLBACK;
  return normalizeFilterDefsObj(raw);
}
function findFilterDef(id){
  return (getFilterDefs().filters || []).find(f => f.id === id) || null;
}
function coerceFilterParamsInPlace(f){
  const def = findFilterDef(f.id);
  if (!def) return;
  const params = def._params || [];
  params.forEach(p => {
    const k = p.k;
    if (!f.params) f.params = {};
    const v = f.params[k];
    if (p.type === 'bool') {
      if (typeof v === 'boolean') return;
      if (typeof v === 'string') f.params[k] = (v.toLowerCase() === 'true');
      else if (typeof v === 'number') f.params[k] = (v !== 0);
      else f.params[k] = !!v;
    } else if (p.type === 'int') {
      const n = Number(v);
      f.params[k] = Number.isFinite(n) ? Math.trunc(n) : (p.default ?? 0);
      if (p.min !== undefined) f.params[k] = Math.max(p.min, f.params[k]);
      if (p.max !== undefined) f.params[k] = Math.min(p.max, f.params[k]);
    } else if (p.type === 'float') {
      const n = Number(v);
      f.params[k] = Number.isFinite(n) ? n : (p.default ?? 0.0);
      if (p.min !== undefined) f.params[k] = Math.max(p.min, f.params[k]);
      if (p.max !== undefined) f.params[k] = Math.min(p.max, f.params[k]);
    } else if (p.type === 'enum') {
      if (v === undefined || v === null) f.params[k] = (p.default ?? ((p.values||[])[0] ?? ''));
      else f.params[k] = String(v);
      if (Array.isArray(p.values) && p.values.length && !p.values.includes(f.params[k])) {
        f.params[k] = p.default ?? p.values[0];
      }
    } else if (p.type === 'string') {
      f.params[k] = (v === undefined || v === null) ? '' : String(v);
    }
  });
}
function makeDefaultFilterInstance(id){
  const def = findFilterDef(id);
  if (!def) return { id, enabled:true, params:{} };
  const params = {};
  (def._params || []).forEach(p => {
    if (p.default !== undefined) params[p.k] = p.default;
    else if (p.type === 'bool') params[p.k] = false;
    else if (p.type === 'int') params[p.k] = 0;
    else if (p.type === 'float') params[p.k] = 0.0;
    else if (p.type === 'string') params[p.k] = "";
    else if (p.type === 'enum') params[p.k] = (p.values && p.values[0]) ? p.values[0] : "";
  });
  return { id, enabled:true, params };
}
function ensureVideoFilterDefaults(L){
  if (!L.filters || !Array.isArray(L.filters)) L.filters = [];
  L.filters = L.filters.map(f => {
    if (!f || typeof f !== 'object') f = {};
    if (!f.id) f.id = "mono";
    if (f.enabled === undefined) f.enabled = true;
    if (!f.params || typeof f.params !== 'object') f.params = {};
    const def = findFilterDef(f.id);
    if (def) {
      (def._params || []).forEach(p => {
        if (f.params[p.k] === undefined && p.default !== undefined) f.params[p.k] = p.default;
      });
    }
    coerceFilterParamsInPlace(f);
    return f;
  });
  return L;
}
function ensureLayerDefaults(L){
  if (!L.name) L.name = "Layer";
  if (!L.type) L.type = "video";
  if (L.enabled === undefined) L.enabled = true;
  if (!isNum(L.opacity)) L.opacity = 1.0;
  if (!L.invertRel) L.invertRel = "none";
  if (L.type === 'video') {
    if (L.videoSource === undefined) L.videoSource = "";
    if (typeof L.videoSource !== 'string') L.videoSource = String(L.videoSource ?? "");
  }
  if (!L.srcRect || !parseRect(L.srcRect)) L.srcRect = "0,0,1,1";
  if (!L.dstPos  || !parsePos(L.dstPos))   L.dstPos  = "0,0";
  if (!L.scale   || !parseScale(L.scale))  L.scale   = "1.0,1.0";
  if (L.type === 'video') ensureVideoFilterDefaults(L);
  if (!L.crosshair) {
    L.crosshair = {enabled:false, diam:"50x50", center:"", thickness:1, mode:"solid", color:"255,255,255", opacity:1.0, invertRel:"none"};
  } else {
    if (L.crosshair.enabled === undefined) L.crosshair.enabled = false;
    if (!L.crosshair.diam || !parseDimWxH(L.crosshair.diam)) L.crosshair.diam = "50x50";
    if (L.crosshair.center === undefined) L.crosshair.center = "";
    if (!isNum(L.crosshair.thickness)) L.crosshair.thickness = 1;
    L.crosshair.thickness = clamp(Number(L.crosshair.thickness), 1, 99);
    if (!L.crosshair.mode || !['solid','invert'].includes(L.crosshair.mode)) L.crosshair.mode = "solid";
    if (!L.crosshair.color || !parseRGB(L.crosshair.color)) L.crosshair.color = "255,255,255";
    if (!isNum(L.crosshair.opacity)) L.crosshair.opacity = 1.0;
    L.crosshair.opacity = clamp(Number(L.crosshair.opacity), 0, 1);
    if (!L.crosshair.invertRel || !['none','lower','upper'].includes(L.crosshair.invertRel)) L.crosshair.invertRel = "none";
    if (L.crosshair.center && L.crosshair.center.length && !parsePos(L.crosshair.center)) L.crosshair.center = "";
  }
  return L;
}
function normalizeCfg(c){
  c.layers = c.layers || [];
  c.layers = c.layers.map(L => ensureLayerDefaults(L));
  if (!c.present) c.present = "fit";
  if (c.threads === undefined) c.threads = 0;
  if (c.bgr === undefined) c.bgr = false;
  if (c.preferOutputMode === undefined) c.preferOutputMode = true;
  if (!c.preferOutputRes) c.preferOutputRes = "1280x720";
  if (c.viewport === undefined) c.viewport = "";
  if (!Array.isArray(c.v4l2Sources)) c.v4l2Sources = [];
  c.v4l2Sources = c.v4l2Sources.map(String).filter(s => s.length);
  return c;
}
function setMode(m){
  ui.mode = m;
  document.getElementById('pillMode').textContent = `mode: ${m}`;
  document.getElementById('cropPanel').style.display = (m === 'crop') ? 'block' : 'none';
  if (m !== 'crop') {
    ui.crop.active = false;
    ui.crop.rect = null;
    ui.crop.drag = null;
  }
  renderAll();
}
function getViewportWH(){
  let vpw=0,vph=0;
  const viewportStr = (cfg && typeof cfg.viewport === 'string') ? cfg.viewport : '';
  const d = parseDimWxH(viewportStr);
  if (d) { vpw=d.w; vph=d.h; }
  if (!vpw || !vph) {
    vpw = stage.imgNaturalW || 1920;
    vph = stage.imgNaturalH || 1080;
  }
  vpw = clamp(vpw, 1, LIMITS.dim.max);
  vph = clamp(vph, 1, LIMITS.dim.max);
  return {vpw,vph};
}
function updateStageMetrics(){
  const img = document.getElementById('refImg');
  if (!img) return;
  const rect = img.getBoundingClientRect();
  stage.stageW = rect.width || stage.stageW || 1;
  stage.stageH = rect.height || stage.stageH || 1;
  const v = getViewportWH();
  stage.vpw = v.vpw;
  stage.vph = v.vph;
}
function vpToStageX(v){ return v * (stage.stageW / stage.vpw); }
function vpToStageY(v){ return v * (stage.stageH / stage.vph); }
function stageToVpX(px){ return px * (stage.vpw / stage.stageW); }
function stageToVpY(px){ return px * (stage.vph / stage.stageH); }

/* ---------------- Crop helpers (added) ---------------- */
function rectIntersect(a,b){
  const x0 = Math.max(a.x, b.x);
  const y0 = Math.max(a.y, b.y);
  const x1 = Math.min(a.x + a.w, b.x + b.w);
  const y1 = Math.min(a.y + a.h, b.y + b.h);
  const w = Math.max(0, x1 - x0);
  const h = Math.max(0, y1 - y0);
  return {x:x0, y:y0, w, h};
}
function rectEmpty(r){ return !r || r.w <= 0 || r.h <= 0; }
function fmtRect(r){ return `${round(r.x)},${round(r.y)},${round(r.w)},${round(r.h)}`; }

// Convert a crop rectangle expressed in *viewport coords within the layer box*
// into a srcRect in *source pixel coords*.
function cropVPRectToSrcRect(L, cropInLayerVP){
  const sr = parseRect(L.srcRect);
  const sc = parseScale(L.scale);
  if (!sr || !sc) return null;

  // layer box size in VP is sr.w*sc.sx, sr.h*sc.sy
  // so src pixels = vp / scale
  const x = sr.x + (cropInLayerVP.x / sc.sx);
  const y = sr.y + (cropInLayerVP.y / sc.sy);
  const w = (cropInLayerVP.w / sc.sx);
  const h = (cropInLayerVP.h / sc.sy);

  // clamp + round
  const xi = Math.max(0, Math.floor(x + 0.5));
  const yi = Math.max(0, Math.floor(y + 0.5));
  const wi = Math.max(1, Math.floor(w + 0.5));
  const hi = Math.max(1, Math.floor(h + 0.5));
  return {x:xi, y:yi, w:wi, h:hi};
}

/* ---------------------------------------------------- */

function computeLayerBoxVP(L){
  const dp = parsePos(L.dstPos) || {x:0,y:0};
  const sr = parseRect(L.srcRect) || {x:0,y:0,w:1,h:1};
  const sc = parseScale(L.scale) || {sx:1,sy:1};
  let w = Math.max(1, sr.w * sc.sx);
  let h = Math.max(1, sr.h * sc.sy);
  if (!isFinite(w) || w < 1) w = 1;
  if (!isFinite(h) || h < 1) h = 1;
  return {x: dp.x, y: dp.y, w, h};
}
function writeLayerFromBoxVP(L, box){
  const sr = parseRect(L.srcRect) || {x:0,y:0,w:1,h:1};
  const sw = Math.max(1, sr.w);
  const sh = Math.max(1, sr.h);
  const sx = box.w / sw;
  const sy = box.h / sh;
  L.dstPos = `${round(box.x)},${round(box.y)}`;
  L.scale = `${fmt(sx)},${fmt(sy)}`;
}
function snap(v, step, enabled){
  if (!enabled) return v;
  return Math.round(v / step) * step;
}
function renderVPHelp(){
  const el = document.getElementById('vpHelp');
  if (!el) return;
  const s = document.getElementById('viewport') ? document.getElementById('viewport').value.trim() : '';
  if (!s) {
    el.textContent = `Using input resolution (fallback: ref image ${stage.imgNaturalW||'?'}×${stage.imgNaturalH||'?'})`;
    return;
  }
  const d = parseDimWxH(s);
  if (!d) {
    el.innerHTML = `<span style="color: var(--bad);">Invalid viewport. Use "WxH" like 1920x1080.</span>`;
    return;
  }
  el.innerHTML = `Viewport coordinate space: <strong>${d.w}×${d.h}</strong>`;
}
function clampLayerSrcRectToCaptureProxy(L){
  const sr = parseRect(L.srcRect);
  if (!sr) return;
  const W = stage.imgNaturalW || 0;
  const H = stage.imgNaturalH || 0;
  if (!W || !H) return;
  let x = sr.x, y = sr.y, w = sr.w, h = sr.h;
  w = Math.max(1, w);
  h = Math.max(1, h);
  x = clamp(x, 0, W-1);
  y = clamp(y, 0, H-1);
  if (x + w > W) w = Math.max(1, W - x);
  if (y + h > H) h = Math.max(1, H - y);
  L.srcRect = `${round(x)},${round(y)},${round(w)},${round(h)}`;
}
function nextVideoLayerName(){
  const used = new Set();
  (cfg && cfg.layers ? cfg.layers : []).forEach(L => {
    const nm = (L && typeof L.name === 'string') ? L.name : '';
    const m = nm.match(/^Video\s+(\d+)$/i);
    if (m) used.add(Number(m[1]));
  });
  let n = 1;
  while (used.has(n)) n++;
  return `Video ${n}`;
}
function makeDefaultVideoLayer(){
  const W = stage.imgNaturalW || 0;
  const H = stage.imgNaturalH || 0;
  const srcRect = (W > 0 && H > 0) ? `0,0,${round(W)},${round(H)}` : "0,0,1,1";
  const L = {
    name: nextVideoLayerName(),
    type: "video",
    enabled: true,
    opacity: 1.0,
    invertRel: "none",
    srcRect,
    dstPos: "0,0",
    scale: "1.0,1.0",
    filters: [],
    videoSource: ""
  };
  ensureLayerDefaults(L);
  return L;
}
function stageClientToVP(ev){
  const img = document.getElementById('refImg');
  const r = img.getBoundingClientRect();
  const vx = stageToVpX(ev.clientX - r.left);
  const vy = stageToVpY(ev.clientY - r.top);
  return {x: vx, y: vy};
}
function getLayerFromEventTarget(t){
  while (t && t !== document.body) {
    if (t.dataset && t.dataset.layerIdx !== undefined) {
      const idx = Number(t.dataset.layerIdx);
      if (isFinite(idx)) return idx;
    }
    t = t.parentElement;
  }
  return -1;
}
function startMoveOrResize(ev){
  if (!cfg) return false;
  const idx = getLayerFromEventTarget(ev.target);
  if (idx < 0 || idx >= cfg.layers.length) return false;
  const L = cfg.layers[idx];
  if (!L || L.type !== 'video') return false;
  selectedIdx = idx;
  if (ui.mode === 'crop') return false;
  const handle = (ev.target && ev.target.dataset && ev.target.dataset.handle) ? ev.target.dataset.handle : null;
  const inner = (ev.target && ev.target.classList && ev.target.classList.contains('inner'));
  if (!handle && !inner) return false;
  const p = stageClientToVP(ev);
  const startBox = computeLayerBoxVP(L);
  stageDrag = {
    kind: handle ? 'resize' : 'move',
    layerIdx: idx,
    handle: handle || '',
    startVPX: p.x,
    startVPY: p.y,
    startBox: startBox,
    snap: ev.shiftKey,
  };
  return true;
}
function applyStageDrag(ev){
  if (!stageDrag || !cfg) return;
  const idx = stageDrag.layerIdx;
  const L = cfg.layers[idx];
  if (!L) return;
  const p = stageClientToVP(ev);
  const dxVP = p.x - stageDrag.startVPX;
  const dyVP = p.y - stageDrag.startVPY;
  const snapOn = stageDrag.snap || ev.shiftKey;
  const dx = snap(dxVP, 10, snapOn);
  const dy = snap(dyVP, 10, snapOn);
  let b = {...stageDrag.startBox};
  const minW = 1, minH = 1;
  if (stageDrag.kind === 'move') {
    b.x += dx;
    b.y += dy;
  } else {
    const edge = stageDrag.handle;
    if (edge.includes('n')) { b.y += dy; b.h -= dy; }
    if (edge.includes('s')) { b.h += dy; }
    if (edge.includes('w')) { b.x += dx; b.w -= dx; }
    if (edge.includes('e')) { b.w += dx; }
    b.w = Math.max(minW, b.w);
    b.h = Math.max(minH, b.h);
  }
  writeLayerFromBoxVP(L, b);
  if (selectedIdx === idx) {
    if (document.getElementById('iDstPos')) document.getElementById('iDstPos').value = L.dstPos;
    if (document.getElementById('iScale')) document.getElementById('iScale').value = L.scale;
  }
  renderStage();
  renderCrosshairPreview();
}
function stopStageDrag(){
  if (!stageDrag) return;
  stageDrag = null;
  validateCfg();
  renderLayersTable();
  renderInspector();
  renderCrosshairPreview();
}

/* ---------------- Crop commit logic (added) ---------------- */
function doCropKeep(layerIdx, cropSrcRect){
  const L = cfg.layers[layerIdx];
  if (!L || L.type !== 'video') return;

  // Keep visual placement: move dstPos by crop offset in output-space
  const dp = parsePos(L.dstPos) || {x:0,y:0};
  const sc = parseScale(L.scale) || {sx:1,sy:1};
  const srOld = parseRect(L.srcRect);
  if (!srOld) return;

  const dxSrc = cropSrcRect.x - srOld.x;
  const dySrc = cropSrcRect.y - srOld.y;

  dp.x += dxSrc * sc.sx;
  dp.y += dySrc * sc.sy;

  L.dstPos = `${round(dp.x)},${round(dp.y)}`;
  L.srcRect = fmtRect(cropSrcRect);
}

function doCropRemoveSplit(layerIdx, cropSrcRect){
  const L = cfg.layers[layerIdx];
  if (!L || L.type !== 'video') return;

  const sr = parseRect(L.srcRect);
  const dp = parsePos(L.dstPos) || {x:0,y:0};
  const sc = parseScale(L.scale) || {sx:1,sy:1};
  if (!sr || !sc) return;

  // Clamp crop rect to sr
  const crop = rectIntersect(sr, cropSrcRect);
  if (rectEmpty(crop)) return;

  // Remaining regions in source coords (up to 4)
  const regions = [];

  // Top band
  if (crop.y > sr.y) {
    regions.push({x: sr.x, y: sr.y, w: sr.w, h: crop.y - sr.y});
  }
  // Bottom band
  const cropBottom = crop.y + crop.h;
  const srBottom = sr.y + sr.h;
  if (cropBottom < srBottom) {
    regions.push({x: sr.x, y: cropBottom, w: sr.w, h: srBottom - cropBottom});
  }
  // Middle left
  if (crop.x > sr.x) {
    regions.push({x: sr.x, y: crop.y, w: crop.x - sr.x, h: crop.h});
  }
  // Middle right
  const cropRight = crop.x + crop.w;
  const srRight = sr.x + sr.w;
  if (cropRight < srRight) {
    regions.push({x: cropRight, y: crop.y, w: srRight - cropRight, h: crop.h});
  }

  const baseName = L.name || "Layer";
  const baseVideoSource = (L.videoSource !== undefined) ? L.videoSource : "";
  const baseInvert = L.invertRel || "none";
  const baseOpacity = isNum(L.opacity) ? L.opacity : 1.0;
  const baseEnabled = (L.enabled !== undefined) ? !!L.enabled : true;
  const baseFilters = Array.isArray(L.filters) ? deepClone(L.filters) : [];
  const baseScale = L.scale;

  const newLayers = regions
    .filter(r => r.w > 0 && r.h > 0)
    .map((r, i) => {
      const dx = (r.x - sr.x) * sc.sx;
      const dy = (r.y - sr.y) * sc.sy;
      return ensureLayerDefaults({
        name: `${baseName}_split${i+1}`,
        type: "video",
        enabled: baseEnabled,
        opacity: baseOpacity,
        invertRel: baseInvert,
        srcRect: fmtRect(r),
        dstPos: `${round(dp.x + dx)},${round(dp.y + dy)}`,
        scale: baseScale,
        filters: baseFilters,
        videoSource: baseVideoSource,
      });
    });

  if (newLayers.length === 0) {
    L.enabled = false;
    return;
  }

  cfg.layers.splice(layerIdx, 1, ...newLayers);
  selectedIdx = layerIdx;
}

function hookCropPanel(){
  const btnCancel = document.getElementById('btnCropCancel');
  const btnCommit = document.getElementById('btnCropCommit');

  btnCancel.onclick = () => {
    setMode('select');
    const read = document.getElementById('cropReadout');
    if (read) read.textContent = '(none)';
  };

  btnCommit.onclick = () => {
    if (!cfg || selectedIdx < 0 || selectedIdx >= cfg.layers.length) return;
    const L = cfg.layers[selectedIdx];
    if (!L || L.type !== 'video') return;
    if (!ui.crop.rect || ui.crop.rect.w < 1 || ui.crop.rect.h < 1) {
      alert("Draw a crop rectangle first.");
      return;
    }

    const cropSrc = cropVPRectToSrcRect(L, ui.crop.rect);
    if (!cropSrc) {
      alert("Invalid crop rectangle.");
      return;
    }

    const mode = document.getElementById('cropMode').value;
    if (mode === 'keep') doCropKeep(selectedIdx, cropSrc);
    else if (mode === 'removeSplit') doCropRemoveSplit(selectedIdx, cropSrc);

    ui.crop.rect = null;
    validateCfg();
    setMode('select');
    renderAll();
  };
}
/* -------------------------------------------------------- */

function hookStagePointer(){
  const stageEl = document.getElementById('overlayStage');

  stageEl.onpointerdown = (ev) => {
    if (!cfg) return;

    const idx = getLayerFromEventTarget(ev.target);
    if (idx >= 0) {
      selectedIdx = idx;
      renderLayersTable();
      renderInspector();
      renderCrosshairPreview();

      // Crop mode: start drawing crop rect inside selected video layer
      if (ui.mode === 'crop') {
        const L = cfg.layers[selectedIdx];
        if (!L || L.type !== 'video') return;

        updateStageMetrics();
        const p = stageClientToVP(ev);
        const box = computeLayerBoxVP(L);

        // Only start if click is inside the layer box
        if (p.x < box.x || p.y < box.y || p.x > box.x + box.w || p.y > box.y + box.h) return;

        ui.crop.active = true;
        ui.crop.drag = {
          startVP: {x: p.x, y: p.y},
          curVP: {x: p.x, y: p.y},
        };
        ui.crop.rect = {x:0,y:0,w:0,h:0};

        stageEl.setPointerCapture(ev.pointerId);
        ev.preventDefault();
        renderStage();
        return;
      }

      // Normal move/resize mode
      if (startMoveOrResize(ev)) {
        stageEl.setPointerCapture(ev.pointerId);
        ev.preventDefault();
        return;
      }
    } else {
      selectedIdx = -1;
      renderLayersTable();
      renderInspector();
      renderCrosshairPreview();
    }
  };

  stageEl.onpointermove = (ev) => {
    if (!cfg) return;

    if (ui.mode === 'crop' && ui.crop.active && ui.crop.drag && selectedIdx >= 0) {
      const L = cfg.layers[selectedIdx];
      if (!L || L.type !== 'video') return;

      const p = stageClientToVP(ev);
      ui.crop.drag.curVP = {x:p.x, y:p.y};

      const box = computeLayerBoxVP(L);

      // Convert dragged endpoints into a rect in *layer-local VP coords*
      const x0 = clamp(Math.min(ui.crop.drag.startVP.x, p.x), box.x, box.x + box.w);
      const y0 = clamp(Math.min(ui.crop.drag.startVP.y, p.y), box.y, box.y + box.h);
      const x1 = clamp(Math.max(ui.crop.drag.startVP.x, p.x), box.x, box.x + box.w);
      const y1 = clamp(Math.max(ui.crop.drag.startVP.y, p.y), box.y, box.y + box.h);

      ui.crop.rect = {
        x: x0 - box.x,
        y: y0 - box.y,
        w: Math.max(0, x1 - x0),
        h: Math.max(0, y1 - y0),
      };

      const sr = cropVPRectToSrcRect(L, ui.crop.rect);
      const read = document.getElementById('cropReadout');
      if (read) read.textContent = sr ? fmtRect(sr) : '(invalid)';

      renderStage();
      ev.preventDefault();
      return;
    }

    if (!stageDrag) return;
    ev.preventDefault();
    applyStageDrag(ev);
  };

  stageEl.onpointerup = () => {
    if (ui.mode === 'crop') {
      ui.crop.active = false;
      ui.crop.drag = null;
      renderStage();
      return;
    }
    stopStageDrag();
  };
  stageEl.onpointercancel = () => {
    if (ui.mode === 'crop') {
      ui.crop.active = false;
      ui.crop.drag = null;
      renderStage();
      return;
    }
    stopStageDrag();
  };
}

function renderStage(){
  if (!cfg) return;
  updateStageMetrics();
  const stageEl = document.getElementById('overlayStage');
  stageEl.style.width = stage.stageW + "px";
  stageEl.style.height = stage.stageH + "px";
  stageEl.innerHTML = '';
  cfg.layers.forEach((L, idx) => {
    ensureLayerDefaults(L);
    if (L.type !== 'video') return;
    const boxVP = computeLayerBoxVP(L);
    const el = document.createElement('div');
    el.className = 'box' + (idx === selectedIdx ? ' selected' : '') + (!L.enabled ? ' disabled' : '');
    el.dataset.layerIdx = String(idx);
    el.style.left = vpToStageX(boxVP.x) + 'px';
    el.style.top  = vpToStageY(boxVP.y) + 'px';
    el.style.width  = vpToStageX(boxVP.w) + 'px';
    el.style.height = vpToStageY(boxVP.h) + 'px';
    el.style.opacity = String(clamp(Number(L.opacity ?? 1),0,1));
    const lab = document.createElement('div');
    lab.className = 'label';
    const src = (L.videoSource && L.videoSource.length) ? L.videoSource : (cfg.v4l2Dev || '');
    lab.textContent = `${idx}: ${L.name||'Layer'} ${src ? '['+src+']' : ''}`;
    el.appendChild(lab);
    const inner = document.createElement('div');
    inner.className = 'inner';
    inner.dataset.layerIdx = String(idx);
    el.appendChild(inner);
    const handles = ['nw','n','ne','e','se','s','sw','w'];
    handles.forEach(h => {
      const hd = document.createElement('div');
      hd.className = 'handle h-' + h;
      hd.dataset.layerIdx = String(idx);
      hd.dataset.handle = h;
      el.appendChild(hd);
    });
    stageEl.appendChild(el);
  });

  // Crop overlay (added)
  if (ui.mode === 'crop' && selectedIdx >= 0 && selectedIdx < cfg.layers.length) {
    const L = cfg.layers[selectedIdx];
    if (L && L.type === 'video' && ui.crop.rect && ui.crop.rect.w > 0 && ui.crop.rect.h > 0) {
      const boxVP = computeLayerBoxVP(L);

      const cropEl = document.createElement('div');
      cropEl.className = 'cropRect';
      cropEl.style.left = vpToStageX(boxVP.x + ui.crop.rect.x) + 'px';
      cropEl.style.top  = vpToStageY(boxVP.y + ui.crop.rect.y) + 'px';
      cropEl.style.width  = vpToStageX(ui.crop.rect.w) + 'px';
      cropEl.style.height = vpToStageY(ui.crop.rect.h) + 'px';

      const cap = document.createElement('div');
      cap.className = 'cap';
      const sr = cropVPRectToSrcRect(L, ui.crop.rect);
      cap.textContent = sr ? `srcRect => ${fmtRect(sr)}` : 'invalid crop';
      cropEl.appendChild(cap);

      stageEl.appendChild(cropEl);
    }
  }
}

function renderCrosshairPreview(){
  if (!cfg) return;
  // (frontend preview optional; keeping stub)
}
function renderLayersTable(){
  if (!cfg) return;
  const tb = document.getElementById('layersTable');
  tb.innerHTML = '';
  document.getElementById('layerCount').textContent = `${(cfg.layers||[]).length}`;
  (cfg.layers||[]).forEach((L, idx) => {
    ensureLayerDefaults(L);
    const tr = document.createElement('tr');
    const isSel = idx === selectedIdx;
    const name = (L.name||'').replaceAll('"','&quot;');
    const type = L.type || 'video';
    const inv = L.invertRel || 'none';
    const op = isNum(L.opacity) ? L.opacity : 1.0;
    tr.innerHTML = `
      <td class="right mono">${idx}</td>
      <td><input data-k="name" value="${name}" style="${isSel?'border-color: rgba(255,209,102,0.65);':''}"/></td>
      <td class="mono">${type}</td>
      <td><input data-k="enabled" type="checkbox" ${L.enabled ? 'checked':''}/></td>
      <td><input data-k="opacity" type="number" min="0" max="1" step="0.01" value="${op}"/></td>
      <td>
        <select data-k="invertRel">
          <option value="none" ${inv==='none'?'selected':''}>none</option>
          <option value="lower" ${inv==='lower'?'selected':''}>lower</option>
          <option value="upper" ${inv==='upper'?'selected':''}>upper</option>
        </select>
      </td>
      <td class="right">
        <div class="rowBtns" style="justify-content:flex-end;">
          <button data-act="sel">${isSel?'Selected':'Select'}</button>
          <button data-act="dup">Dup</button>
          <button data-act="del">Del</button>
        </div>
      </td>
    `;
    tr.querySelectorAll('input,select').forEach(inp => {
      inp.onchange = () => {
        const k = inp.getAttribute('data-k');
        if (k==='enabled') L.enabled = inp.checked;
        else if (k==='opacity') L.opacity = clamp(Number(inp.value), 0, 1);
        else if (k==='name') L.name = inp.value;
        else if (k==='invertRel') L.invertRel = inp.value;
        validateCfg();
        renderStage();
        renderInspector();
      };
    });
    tr.querySelectorAll('button').forEach(btn => {
      btn.onclick = () => {
        const a = btn.getAttribute('data-act');
        if (a==='sel'){ selectedIdx = idx; setMode('select'); renderAll(); }
        if (a==='del'){
          cfg.layers.splice(idx,1);
          if(selectedIdx===idx) selectedIdx=-1;
          if(selectedIdx>idx) selectedIdx--;
          setMode('select');
          renderAll();
        }
        if (a==='dup'){
          const copy = deepClone(cfg.layers[idx]);
          copy.name = (copy.name||'Layer') + "_copy";
          cfg.layers.splice(idx+1, 0, copy);
          selectedIdx = idx+1;
          renderAll();
        }
      };
    });
    tb.appendChild(tr);
  });
}
function renderFilterAddSelect(){
  const sel = document.getElementById('fAddType');
  if (!sel) return;
  const defs = getFilterDefs().filters || [];
  sel.innerHTML = '';
  defs.forEach(d => {
    const opt = document.createElement('option');
    opt.value = d.id;
    opt.textContent = `${d.name} (${d.id})`;
    sel.appendChild(opt);
  });
}
function renderFiltersUIForLayer(L){
  const box = document.getElementById('filtersBox');
  const chip = document.getElementById('fCountChip');
  const addSel = document.getElementById('fAddType');
  const addBtn = document.getElementById('btnAddFilter');
  if (!box || !chip || !addSel || !addBtn) return;
  if (!L || L.type !== 'video') {
    box.innerHTML = '';
    chip.textContent = '0';
    addBtn.disabled = true;
    addSel.disabled = true;
    return;
  }
  addBtn.disabled = false;
  addSel.disabled = false;
  ensureVideoFilterDefaults(L);
  chip.textContent = `${L.filters.length}`;
  renderFilterAddSelect();
  addBtn.onclick = () => {
    const id = addSel.value;
    L.filters.push(makeDefaultFilterInstance(id));
    validateCfg();
    renderInspector();
  };
  box.innerHTML = '';
  L.filters.forEach((f, idx) => {
    coerceFilterParamsInPlace(f);
    const def = findFilterDef(f.id);
    const name = def ? def.name : f.id;
    const el = document.createElement('div');
    el.className = 'fItem';
    el.innerHTML = `
      <div class="top">
        <div class="left">
          <span class="name">${name}</span>
          <span class="chip mono">#${idx}</span>
          <span class="chip mono">${f.id}</span>
          <label class="mini"><input type="checkbox" data-k="enabled" ${f.enabled ? 'checked':''}/> enabled</label>
        </div>
        <div class="rowBtns">
          <button class="secondary" data-act="up">Up</button>
          <button class="secondary" data-act="dn">Down</button>
          <button class="danger" data-act="del">Remove</button>
        </div>
      </div>
      <div class="params"></div>
    `;
    const paramsEl = el.querySelector('.params');
    const params = def ? (def._params || []) : [];
    if (params.length === 0) {
      paramsEl.innerHTML = `<div class="help mono">no params</div>`;
    } else {
      params.forEach(p => {
        const wrap = document.createElement('div');
        const key = p.k;
        const label = document.createElement('label');
        label.textContent = key;
        wrap.appendChild(label);
        let input;
        if (p.type === 'enum') {
          input = document.createElement('select');
          (p.values || []).forEach(v => {
            const opt = document.createElement('option');
            opt.value = v;
            opt.textContent = v;
            if (String(f.params[key] ?? p.default ?? '') === String(v)) opt.selected = true;
            input.appendChild(opt);
          });
        } else if (p.type === 'bool') {
          input = document.createElement('input');
          input.type = 'checkbox';
          input.checked = !!(f.params[key] ?? p.default ?? false);
        } else if (p.type === 'int') {
          input = document.createElement('input');
          input.type = 'number';
          if (p.min !== undefined) input.min = String(p.min);
          if (p.max !== undefined) input.max = String(p.max);
          input.step = '1';
          input.value = String(f.params[key] ?? p.default ?? 0);
        } else if (p.type === 'float') {
          input = document.createElement('input');
          input.type = 'number';
          if (p.min !== undefined) input.min = String(p.min);
          if (p.max !== undefined) input.max = String(p.max);
          input.step = '0.01';
          input.value = String(f.params[key] ?? p.default ?? 0.0);
        } else {
          input = document.createElement('input');
          input.type = 'text';
          input.value = String(f.params[key] ?? p.default ?? '');
        }
        input.onchange = () => {
          if (p.type === 'bool') f.params[key] = !!input.checked;
          else if (p.type === 'int') f.params[key] = Number(input.value);
          else if (p.type === 'float') f.params[key] = Number(input.value);
          else f.params[key] = input.value;
          coerceFilterParamsInPlace(f);
          validateCfg();
        };
        wrap.appendChild(input);
        paramsEl.appendChild(wrap);
      });
    }
    el.querySelector('input[data-k="enabled"]').onchange = (ev) => {
      f.enabled = !!ev.target.checked;
      validateCfg();
    };
    el.querySelectorAll('button').forEach(btn => {
      btn.onclick = () => {
        const act = btn.getAttribute('data-act');
        if (act === 'del') {
          L.filters.splice(idx,1);
          validateCfg();
          renderInspector();
          return;
        }
        if (act === 'up' && idx > 0) {
          const tmp = L.filters[idx-1];
          L.filters[idx-1] = L.filters[idx];
          L.filters[idx] = tmp;
          validateCfg();
          renderInspector();
          return;
        }
        if (act === 'dn' && idx < L.filters.length-1) {
          const tmp = L.filters[idx+1];
          L.filters[idx+1] = L.filters[idx];
          L.filters[idx] = tmp;
          validateCfg();
          renderInspector();
          return;
        }
      };
    });
    box.appendChild(el);
  });
}
function getKnownSources(){
  const set = new Set();
  const arr = [];
  const add = (s) => {
    if (!s || !String(s).trim()) return;
    const v = String(s).trim();
    if (set.has(v)) return;
    set.add(v);
    arr.push(v);
  };
  if (cfg && cfg.v4l2Dev) add(cfg.v4l2Dev);
  if (cfg && Array.isArray(cfg.v4l2Sources)) cfg.v4l2Sources.forEach(add);
  const rtSources = runtime && runtime.runtime && Array.isArray(runtime.runtime.sources) ? runtime.runtime.sources : [];
  rtSources.forEach(x => add(x.dev));
  add("/dev/v4l/by-path/platform-fe800000.csi-video-index0");
  add("/dev/v4l/by-path/platform-fe801000.csi-video-index0");
  return arr;
}
function getActiveSourcesFromRuntime(){
  const rtSources = runtime && runtime.runtime && Array.isArray(runtime.runtime.sources) ? runtime.runtime.sources : [];
  return rtSources.filter(s => !!s.active).map(s => s.dev);
}
function renderVideoSourceSelectForLayer(L){
  const sel = document.getElementById('iVideoSource');
  const help = document.getElementById('iVideoSourceHelp');
  if (!sel || !help) return;
  if (!L || L.type !== 'video') {
    sel.innerHTML = '';
    sel.disabled = true;
    help.textContent = '';
    return;
  }
  sel.disabled = false;
  const known = getKnownSources();
  const active = new Set(getActiveSourcesFromRuntime());
  const cur = (L.videoSource && L.videoSource.length) ? L.videoSource : "";
  sel.innerHTML = '';
  {
    const opt = document.createElement('option');
    opt.value = "";
    const def = (cfg && cfg.v4l2Dev) ? cfg.v4l2Dev : "";
    opt.textContent = def ? `default (${def})` : 'default';
    if (cur === "") opt.selected = true;
    sel.appendChild(opt);
  }
  known.forEach(dev => {
    const opt = document.createElement('option');
    opt.value = dev;
    opt.textContent = active.has(dev) ? `${dev} (active)` : `${dev}`;
    if (cur === dev) opt.selected = true;
    sel.appendChild(opt);
  });
  if (cur && !known.includes(cur)) {
    const opt = document.createElement('option');
    opt.value = cur;
    opt.textContent = `${cur} (custom)`;
    opt.selected = true;
    sel.appendChild(opt);
  }
  sel.onchange = () => {
    L.videoSource = sel.value;
    validateCfg();
    renderStage();
    renderInspector();
  };
  const effective = cur ? cur : (cfg && cfg.v4l2Dev ? cfg.v4l2Dev : "");
  if (!effective) {
    help.textContent = "No default source configured.";
  } else if (active.size) {
    help.textContent = active.has(effective)
      ? `Effective source ${effective} is active.`
      : `Effective source ${effective} is not active (layer will be auto-disabled by backend).`;
  } else {
    help.textContent = `Effective source ${effective}. (No runtime source status yet.)`;
  }
}
function renderInspector(){
  if (!cfg) return;
  const chip = document.getElementById('inspectorChip');
  const help = document.getElementById('inspectorHelp');
  const body = document.getElementById('inspectorBody');
  const vid = document.getElementById('videoInspector');
  const xh = document.getElementById('crossInspector');
  if (selectedIdx < 0 || selectedIdx >= cfg.layers.length) {
    chip.textContent = 'no selection';
    help.textContent = 'Select a layer by clicking it in the viewport or in the table.';
    body.style.display = 'none';
    return;
  }
  const L = cfg.layers[selectedIdx];
  ensureLayerDefaults(L);
  chip.textContent = `#${selectedIdx} ${L.name || 'Layer'} (${L.type})`;
  help.textContent = (L.type === 'video')
    ? 'Drag/resize in viewport; dstPos/scale update live.'
    : (L.type === 'crosshair')
      ? 'Crosshair preview shown in the viewport.'
      : 'Layer type not fully implemented yet.';
  body.style.display = 'block';
  document.getElementById('iName').value = L.name || '';
  document.getElementById('iType').value = L.type || 'video';
  document.getElementById('iEnabled').checked = !!L.enabled;
  document.getElementById('iOpacity').value = clamp(Number(L.opacity ?? 1),0,1);
  document.getElementById('iInvertRel').value = L.invertRel || 'none';
  vid.style.display = (L.type === 'video') ? 'block' : 'none';
  xh.style.display  = (L.type === 'crosshair') ? 'block' : 'none';
  if (L.type === 'video') {
    renderVideoSourceSelectForLayer(L);
    document.getElementById('iSrcRect').value = L.srcRect || '0,0,1,1';
    document.getElementById('iDstPos').value = L.dstPos || '0,0';
    document.getElementById('iScale').value = L.scale || '1.0,1.0';
    renderFiltersUIForLayer(L);
  }
}
function validateCfg(){
  if (!cfg) return [];
  const errors = [];
  if (cfg.v4l2Sources && !Array.isArray(cfg.v4l2Sources)) errors.push("v4l2Sources must be array");
  cfg.layers.forEach((L, idx) => {
    ensureLayerDefaults(L);
    if (L.type === 'video') {
      if (L.videoSource !== undefined && typeof L.videoSource !== 'string') errors.push(`layer ${idx}: videoSource must be string`);
      ensureVideoFilterDefaults(L);
      L.filters.forEach((f, fi) => {
        if (!f.id || typeof f.id !== 'string') errors.push(`layer ${idx} filter ${fi}: missing id`);
        const def = findFilterDef(f.id);
        if (!def) return;
        (def._params || []).forEach(p => {
          const v = f.params ? f.params[p.k] : undefined;
          if (p.type === 'enum') {
            if (v !== undefined && !(p.values || []).includes(v)) errors.push(`layer ${idx} filter ${fi}: ${p.k} invalid`);
          } else if (p.type === 'int') {
            const n = Number(v);
            if (!isFinite(n)) errors.push(`layer ${idx} filter ${fi}: ${p.k} must be number`);
          } else if (p.type === 'float') {
            const n = Number(v);
            if (!isFinite(n)) errors.push(`layer ${idx} filter ${fi}: ${p.k} must be number`);
          }
        });
      });
    }
  });
  const box = document.getElementById('validationBox');
  if (box) {
    if (errors.length === 0) {
      box.textContent = 'ok';
      box.className = 'help mono';
    } else {
      box.textContent = errors.map(e => '• ' + e).join('\n');
      box.className = 'err mono';
    }
  }
  return errors;
}
async function refreshStatus(){
  try {
    const st = await api('/api/status');
    runtime = st || null;
    const s = document.getElementById('status');
    if (s) s.textContent = JSON.stringify(st, null, 2);
    if (cfg && selectedIdx >= 0 && selectedIdx < cfg.layers.length) {
      renderInspector();
    }
  } catch (e) {
    const s = document.getElementById('status');
    if (s) s.textContent = `status error: ${e}`;
  }
}
function setFormFromCfg(c){
  document.getElementById('v4l2Dev').value = c.v4l2Dev||'/dev/v4l/by-path/platform-fe800000.csi-video-index0';
  document.getElementById('v4l2Sources').value = (Array.isArray(c.v4l2Sources) && c.v4l2Sources.length) ? c.v4l2Sources.join(',') : '';
  document.getElementById('edidPath').value = c.edidPath||'';
  document.getElementById('present').value = c.present||'fit';
  document.getElementById('viewport').value = c.viewport||'';
  document.getElementById('preferOutputMode').checked = !!c.preferOutputMode;
  document.getElementById('preferOutputRes').value = c.preferOutputRes||'1280x720';
  document.getElementById('threads').value = c.threads||0;
  document.getElementById('bgr').checked = !!c.bgr;
  renderVPHelp();
}
function cfgFromForm(){
  cfg.layers = (cfg.layers||[]).map(L => ensureLayerDefaults(L));
  (cfg.layers||[]).forEach(L => {
    if (L.type !== 'video') return;
    if (!Array.isArray(L.filters)) return;
    L.filters.forEach(f => coerceFilterParamsInPlace(f));
  });
  const v4l2Sources = parseCSVStrings(document.getElementById('v4l2Sources').value);
  return {
    v4l2Dev: document.getElementById('v4l2Dev').value,
    v4l2Sources,
    edidPath: document.getElementById('edidPath').value,
    present: document.getElementById('present').value,
    viewport: document.getElementById('viewport').value,
    preferOutputMode: document.getElementById('preferOutputMode').checked,
    preferOutputRes: document.getElementById('preferOutputRes').value,
    threads: Number(document.getElementById('threads').value),
    bgr: document.getElementById('bgr').checked,
    layers: cfg.layers||[]
  };
}
async function loadAll(){
  cfg = await api('/api/config');
  cfg = normalizeCfg(cfg);
  if (selectedIdx >= cfg.layers.length) selectedIdx = -1;
  setFormFromCfg(cfg);
  validateCfg();
  renderAll();
}
async function loadFilterDefs(){
  try {
    const j = await api('/api/filters');
    if (j && j.filters && Array.isArray(j.filters)) filterDefs = j;
    else filterDefs = FILTER_FALLBACK;
  } catch {
    filterDefs = FILTER_FALLBACK;
  }
  if (cfg) {
    cfg.layers = (cfg.layers||[]).map(L => ensureLayerDefaults(L));
    renderAll();
  }
}
function hookGlobalUI(){
  document.getElementById('btnReload').onclick = async () => {
    setMode('select');
    await loadAll();
  };
  document.getElementById('btnApply').onclick = async () => {
    const errs = validateCfg();
    if (errs.length) {
      if (!confirm("Config has validation errors. Apply anyway?")) return;
    }
    const payload = cfgFromForm();
    const r = await api('/api/apply', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(payload)
    });
    if (r && r.effectiveConfig) {
      cfg = normalizeCfg(r.effectiveConfig);
      setFormFromCfg(cfg);
      validateCfg();
      renderAll();
    }
    await refreshStatus();
  };
  document.getElementById('btnQuit').onclick = async () => {
    await api('/api/quit', {method:'POST'});
  };
  document.getElementById('btnAddVideo').onclick = () => {
    if (!cfg) return;
    if (!Array.isArray(cfg.layers)) cfg.layers = [];
    const L = makeDefaultVideoLayer();
    cfg.layers.push(L);
    selectedIdx = cfg.layers.length - 1;
    setMode('select');
    validateCfg();
    renderAll();
  };
  document.getElementById('btnAddCrosshair').onclick = () => {
    alert("Crosshair layer UI exists in inspector, but add-crosshair not wired in this webui build.");
  };

  // Crop button (changed from alert to real mode toggle)
  document.getElementById('btnCrop').onclick = () => {
    if (selectedIdx < 0 || !cfg || selectedIdx >= cfg.layers.length) {
      alert("Select a video layer first.");
      return;
    }
    const L = cfg.layers[selectedIdx];
    if (!L || L.type !== 'video') {
      alert("Crop is only supported for video layers.");
      return;
    }
    setMode(ui.mode === 'crop' ? 'select' : 'crop');
    const read = document.getElementById('cropReadout');
    if (read) read.textContent = '(draw a rectangle in the selected layer)';
  };

  // Keyboard shortcuts for crop mode
  window.addEventListener('keydown', (ev) => {
    if (ui.mode !== 'crop') return;
    if (ev.key === 'Escape') {
      document.getElementById('btnCropCancel').click();
      ev.preventDefault();
    } else if (ev.key === 'Enter') {
      document.getElementById('btnCropCommit').click();
      ev.preventDefault();
    }
  });
}
function hookInspector(){
  const byId = (id) => document.getElementById(id);
  const iName = byId('iName');
  const iEnabled = byId('iEnabled');
  const iOpacity = byId('iOpacity');
  const iInvertRel = byId('iInvertRel');
  const btnDupLayer = byId('btnDupLayer');
  const iSrcRect = byId('iSrcRect');
  const iDstPos = byId('iDstPos');
  const iScale = byId('iScale');
  const btnClampSrc = byId('btnClampSrc');
  function curLayer(){
    if (!cfg) return null;
    if (selectedIdx < 0 || selectedIdx >= cfg.layers.length) return null;
    return cfg.layers[selectedIdx];
  }
  iName.onchange = () => {
    const L = curLayer(); if (!L) return;
    L.name = iName.value;
    validateCfg(); renderLayersTable(); renderStage(); renderInspector();
  };
  iEnabled.onchange = () => {
    const L = curLayer(); if (!L) return;
    L.enabled = !!iEnabled.checked;
    validateCfg(); renderLayersTable(); renderStage(); renderInspector();
  };
  iOpacity.onchange = () => {
    const L = curLayer(); if (!L) return;
    L.opacity = clamp(Number(iOpacity.value), 0, 1);
    validateCfg(); renderLayersTable(); renderStage(); renderInspector();
  };
  iInvertRel.onchange = () => {
    const L = curLayer(); if (!L) return;
    L.invertRel = iInvertRel.value;
    validateCfg(); renderLayersTable(); renderStage(); renderInspector();
  };
  btnDupLayer.onclick = () => {
    const L = curLayer(); if (!L) return;
    const copy = deepClone(L);
    copy.name = (copy.name||'Layer') + "_copy";
    cfg.layers.splice(selectedIdx+1, 0, copy);
    selectedIdx = selectedIdx+1;
    validateCfg(); renderAll();
  };
  iSrcRect.onchange = () => {
    const L = curLayer(); if (!L || L.type !== 'video') return;
    if (!parseRect(iSrcRect.value)) return;
    L.srcRect = iSrcRect.value;
    validateCfg(); renderStage();
  };
  iDstPos.onchange = () => {
    const L = curLayer(); if (!L || L.type !== 'video') return;
    if (!parsePos(iDstPos.value)) return;
    L.dstPos = iDstPos.value;
    validateCfg(); renderStage();
  };
  iScale.onchange = () => {
    const L = curLayer(); if (!L || L.type !== 'video') return;
    if (!parseScale(iScale.value)) return;
    L.scale = iScale.value;
    validateCfg(); renderStage();
  };
  btnClampSrc.onclick = () => {
    const L = curLayer(); if (!L || L.type !== 'video') return;
    clampLayerSrcRectToCaptureProxy(L);
    validateCfg(); renderInspector(); renderStage();
  };
  document.getElementById('viewport').oninput = () => {
    if (!cfg) return;
    cfg.viewport = document.getElementById('viewport').value;
    renderVPHelp();
    renderStage();
  };
}
function hookImageNaturalSize(){
  const img = document.getElementById('refImg');
  img.onload = () => {
    stage.imgNaturalW = img.naturalWidth;
    stage.imgNaturalH = img.naturalHeight;
    renderVPHelp();
    renderAll();
  };
}
function renderAll(){
  if (!cfg) return;
  renderLayersTable();
  renderInspector();
  renderStage();
}
setInterval(refreshStatus, 1000);
(async () => {
  hookGlobalUI();
  hookInspector();
  hookImageNaturalSize();
  hookStagePointer();
  hookCropPanel();
  await loadFilterDefs();
  await loadAll();
  await refreshStatus();
})();
</script>
</body>
</html>)HTML";

void webui_start_detached(int port) {
  std::thread([port]() {
    httplib::Server svr;

    svr.Get("/", [](const httplib::Request&, httplib::Response &res) {
      res.set_content(kIndexHtml, "text/html; charset=utf-8");
    });

    svr.Get("/ref.png", [](const httplib::Request&, httplib::Response &res) {
      std::vector<uint8_t> png;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        png = g_ref_png;
      }
      if (png.empty()) {
        res.status = 404;
        res.set_content("no reference frame", "text/plain");
        return;
      }
      res.set_content(reinterpret_cast<const char*>(png.data()), png.size(), "image/png");
    });

    svr.Get("/api/config", [](const httplib::Request&, httplib::Response &res) {
      std::string (*cfgp)() = nullptr;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        cfgp = g_cfg_json;
      }
      if (!cfgp) {
        res.status = 500;
        res.set_content("{\"error\":\"no config provider\"}", "application/json");
        return;
      }
      res.set_content(cfgp(), "application/json");
    });

    svr.Get("/api/status", [](const httplib::Request&, httplib::Response &res) {
      std::string (*st)() = nullptr;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        st = g_status;
      }
      if (!st) {
        res.status = 500;
        res.set_content("{\"error\":\"no status provider\"}", "application/json");
        return;
      }
      res.set_content(st(), "application/json");
    });

    // Optional dynamic filter enumeration
    svr.Get("/api/filters", [](const httplib::Request&, httplib::Response &res) {
      std::string (*fp)() = nullptr;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        fp = g_filter_defs;
      }
      if (!fp) {
        // UI falls back to built-in filter list if 404.
        res.status = 404;
        res.set_content("{\"error\":\"no filter defs provider\"}", "application/json");
        return;
      }
      res.set_content(fp(), "application/json");
    });

    // NEW: V4L2 caps endpoint
    svr.Get("/api/v4l2/caps", [](const httplib::Request &req, httplib::Response &res) {
      std::string (*fn)(const std::string&) = nullptr;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        fn = g_v4l2_caps;
      }
      if (!fn) {
        res.status = 404;
        res.set_content("{\"error\":\"no v4l2 caps provider\"}", "application/json");
        return;
      }
      if (!req.has_param("dev")) {
        res.status = 400;
        res.set_content("{\"error\":\"missing dev\"}", "application/json");
        return;
      }
      std::string dev = req.get_param_value("dev");
      if (dev.empty()) {
        res.status = 400;
        res.set_content("{\"error\":\"missing dev\"}", "application/json");
        return;
      }
      res.set_content(fn(dev), "application/json");
    });

    svr.Post("/api/apply", [](const httplib::Request &req, httplib::Response &res) {
      bool (*apply)(const std::string&, std::string&, int&) = nullptr;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        apply = g_apply;
      }
      if (!apply) {
        res.status = 500;
        res.set_content("{\"error\":\"no apply handler\"}", "application/json");
        return;
      }
      std::string out;
      int st = 200;
      (void)apply(req.body, out, st);
      res.status = st;
      res.set_content(out, "application/json");
    });

    svr.Post("/api/quit", [](const httplib::Request&, httplib::Response &res) {
      std::atomic<bool> *q = nullptr;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        q = g_quit;
      }
      if (q) q->store(true);
      res.set_content("{\"ok\":true}", "application/json");
    });

    std::string addr;
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      addr = g_listen_addr;
    }
    fprintf(stderr, "[webui] listening on %s:%d\n", addr.c_str(), port);
    svr.listen(addr.c_str(), port);
  }).detach();
}

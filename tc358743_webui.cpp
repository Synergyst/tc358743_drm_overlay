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

/*
  Fixes requested here:
    1) Layer boxes were "purely visual": they didn't move/resize reliably because pointer handlers
       were attached to each box, but the stage was rebuilt every move, causing pointer capture loss.
       -> Fix: switch to ONE stage-level pointer handler with stable state. Boxes become render-only.
       -> Also: when dragging/resizing, Inspector dstPos/scale updates live (realtime).
    2) Dragging: click+hold in the inner highlighted zone moves the layer.
       Handles resize from edges/corners.
    3) Crosshair apply failing (and no error): root cause is BACKEND validation:
       apply_from_body() requires for every layer object:
         - srcRect, dstPos, scale are present and parseable (it errors otherwise)
       Your previous UI allowed crosshair layers to omit these or set invalid strings.
       -> Fix: ALWAYS keep srcRect/dstPos/scale valid strings in crosshair layers
          and never allow them to become malformed.
       -> Also: ensure crosshair thickness/diam/color/center are always valid formats on Apply.
       This should stop "apply changes fails when crosshair exists".

  Note: "crosshair not visible on actual video feed" is backend-side rendering behavior (not UI).
        However, the apply failure is UI payload validity, fixed here.
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
    .rowBtns .pill { padding:6px 10px; border:1px solid var(--line); border-radius:999px; font-size:12px; color: var(--muted); background: rgba(0,0,0,0.15); }

    table { width:100%; border-collapse:collapse; font-size:12px; }
    th,td { border-top:1px solid var(--line); padding:8px; vertical-align:top; }
    th { text-align:left; color:var(--muted); font-weight:800; }
    .right { text-align:right; }

    .card { background: var(--panel2); border:1px solid var(--line); border-radius: 12px; padding: 10px; }
    .card .hd2 { display:flex; align-items:center; justify-content:space-between; gap:10px; margin-bottom:8px; }
    .chip { font-size:12px; border:1px solid var(--line); border-radius:999px; padding:4px 8px; color:var(--muted); }
    .err { color:#ffd6d6; background: rgba(255,93,93,0.12); border:1px solid rgba(255,93,93,0.35); border-radius:12px; padding:10px; }
    .help { font-size: 12px; color: var(--muted); line-height: 1.35; }
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
            <input id="v4l2Dev" type="text" placeholder="/dev/video0"/>
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
              Crosshair preview is shown in the viewport. If backend rendering still doesn’t show it on HDMI output,
              that’s backend-side, not UI.
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
let runtime = null; // from /api/status.runtime

let ui = {
  mode: 'select',
  crop: {
    active: false,
    rect: null, // layer-local vp coords
    drag: null, // { kind, handle?, startX,startY,startRect }
  }
};

let stage = {
  vpw: 0,
  vph: 0,
  stageW: 0,
  stageH: 0,
  imgNaturalW: 0,
  imgNaturalH: 0,
};

// Stage-level drag state (stable even when DOM is rerendered)
let stageDrag = null; // { kind:'move'|'resize', layerIdx, handle, startVPX, startVPY, startBox, snap }

function ensureLayerDefaults(L){
  if (!L.name) L.name = "Layer";
  if (!L.type) L.type = "video";
  if (L.enabled === undefined) L.enabled = true;
  if (!isNum(L.opacity)) L.opacity = 1.0;
  if (!L.invertRel) L.invertRel = "none";

  // IMPORTANT: backend apply() requires these to exist and parse for every layer object it sees,
  // including crosshair layers.
  if (!L.srcRect || !parseRect(L.srcRect)) L.srcRect = "0,0,1,1";
  if (!L.dstPos  || !parsePos(L.dstPos))   L.dstPos  = "0,0";
  if (!L.scale   || !parseScale(L.scale))  L.scale   = "1.0,1.0";

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
  const d = parseDimWxH(cfg.viewport);
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
  const rect = img.getBoundingClientRect();
  stage.stageW = rect.width;
  stage.stageH = rect.height;
  const v = getViewportWH();
  stage.vpw = v.vpw;
  stage.vph = v.vph;
}
function vpToStageX(v){ return v * (stage.stageW / stage.vpw); }
function vpToStageY(v){ return v * (stage.stageH / stage.vph); }
function stageToVpX(px){ return px * (stage.vpw / stage.stageW); }
function stageToVpY(px){ return px * (stage.vph / stage.stageH); }

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
  const s = document.getElementById('viewport').value.trim();
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

/*** Stage pointer handling (single, robust) ***/
function stageClientToVP(ev){
  const img = document.getElementById('refImg');
  const r = img.getBoundingClientRect();
  const vx = stageToVpX(ev.clientX - r.left);
  const vy = stageToVpY(ev.clientY - r.top);
  return {x: vx, y: vy};
}

function getLayerFromEventTarget(t){
  // walk up to .box and return layer index
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
  const idx = getLayerFromEventTarget(ev.target);
  if (idx < 0 || idx >= cfg.layers.length) return false;
  const L = cfg.layers[idx];
  if (!L || L.type !== 'video') return false;

  selectedIdx = idx;

  // crop mode: ignore layer move/resize; crop handles managed separately
  if (ui.mode === 'crop') return false;

  const handle = (ev.target && ev.target.dataset && ev.target.dataset.handle) ? ev.target.dataset.handle : null;
  const inner = (ev.target && ev.target.classList && ev.target.classList.contains('inner'));

  // Start drag only if handle or inner region
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
  if (!stageDrag) return;
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

  // live update inspector fields (dstPos/scale) while dragging
  if (selectedIdx === idx) {
    if (document.getElementById('iDstPos')) document.getElementById('iDstPos').value = L.dstPos;
    if (document.getElementById('iScale')) document.getElementById('iScale').value = L.scale;
  }

  // fast render only stage + layers table selection highlight
  renderStage();
  renderCrosshairPreview();
  // do not re-render table each move; too heavy. Just inspector and stage.
}

function stopStageDrag(){
  if (!stageDrag) return;
  stageDrag = null;
  validateCfg();
  renderLayersTable();
  renderInspector();
  renderCrosshairPreview();
}

function hookStagePointer(){
  const stageEl = document.getElementById('overlayStage');

  stageEl.onpointerdown = (ev) => {
    // selecting a layer by clicking its box
    const idx = getLayerFromEventTarget(ev.target);
    if (idx >= 0) {
      selectedIdx = idx;
      renderLayersTable();
      renderInspector();
      renderCrosshairPreview();
      // If we started move/resize, capture pointer
      if (startMoveOrResize(ev)) {
        stageEl.setPointerCapture(ev.pointerId);
        ev.preventDefault();
        return;
      }
    } else {
      // click empty -> deselect
      selectedIdx = -1;
      renderLayersTable();
      renderInspector();
      renderCrosshairPreview();
    }
  };

  stageEl.onpointermove = (ev) => {
    if (!stageDrag) return;
    ev.preventDefault();
    applyStageDrag(ev);
  };

  stageEl.onpointerup = () => stopStageDrag();
  stageEl.onpointercancel = () => stopStageDrag();
}

/*** Crop (kept from previous, but uses selected layer) ***/
function pointerToLayerLocalVP(ev, layerBoxVP){
  const p = stageClientToVP(ev);
  const lx = clamp(p.x - layerBoxVP.x, 0, layerBoxVP.w);
  const ly = clamp(p.y - layerBoxVP.y, 0, layerBoxVP.h);
  return {x: lx, y: ly};
}
function pointInRect(p, r){
  return p.x >= r.x && p.y >= r.y && p.x <= (r.x+r.w) && p.y <= (r.y+r.h);
}
function normalizeRect(r, layerBoxVP){
  let x = r.x, y = r.y, w = r.w, h = r.h;
  if (w < 0) { x += w; w = -w; }
  if (h < 0) { y += h; h = -h; }
  w = Math.max(1, w);
  h = Math.max(1, h);
  x = clamp(x, 0, layerBoxVP.w - 1);
  y = clamp(y, 0, layerBoxVP.h - 1);
  if (x + w > layerBoxVP.w) w = Math.max(1, layerBoxVP.w - x);
  if (y + h > layerBoxVP.h) h = Math.max(1, layerBoxVP.h - y);
  return {x, y, w, h};
}
function updateCropReadout(layerBoxVP){
  const el = document.getElementById('cropReadout');
  if (!ui.crop.rect) { el.textContent='(none)'; return; }
  const r = ui.crop.rect;

  const L = cfg.layers[selectedIdx];
  if (!L) { el.textContent='(none)'; return; }
  const sr = parseRect(L.srcRect);
  const sc = parseScale(L.scale);

  if (!sr || !sc || !(sc.sx>0 && sc.sy>0) || !(sr.w>0 && sr.h>0)) {
    el.textContent = `layer-local vp: x=${round(r.x)}, y=${round(r.y)}, w=${round(r.w)}, h=${round(r.h)} (src mapping unavailable)`;
    return;
  }

  const offXsrc = r.x / sc.sx;
  const offYsrc = r.y / sc.sy;
  const wsrc = r.w / sc.sx;
  const hsrc = r.h / sc.sy;

  el.textContent =
    `layer-local vp: x=${round(r.x)}, y=${round(r.y)}, w=${round(r.w)}, h=${round(r.h)}  |  srcRect add: (${round(offXsrc)},${round(offYsrc)}) size: ${round(wsrc)}x${round(hsrc)}`;
}

function commitCrop(){
  if (ui.mode !== 'crop') return;
  if (selectedIdx < 0 || selectedIdx >= cfg.layers.length) return;
  const L0 = cfg.layers[selectedIdx];
  if (!L0 || L0.type !== 'video') return;
  if (!ui.crop.rect) return;

  const box = computeLayerBoxVP(L0);
  const r = normalizeRect(ui.crop.rect, box);

  const sr0 = parseRect(L0.srcRect);
  const sc0 = parseScale(L0.scale);
  if (!sr0 || !sc0 || !(sr0.w>0 && sr0.h>0) || !(sc0.sx>0 && sc0.sy>0)) {
    alert("Crop requires a valid srcRect and scale.");
    return;
  }

  const cropMode = document.getElementById('cropMode').value;

  if (cropMode === 'keep') {
    const offXsrc = r.x / sc0.sx;
    const offYsrc = r.y / sc0.sy;
    const wsrc = Math.max(1, Math.round(r.w / sc0.sx));
    const hsrc = Math.max(1, Math.round(r.h / sc0.sy));
    const nsr = {
      x: Math.round(sr0.x + offXsrc),
      y: Math.round(sr0.y + offYsrc),
      w: wsrc,
      h: hsrc
    };
    const ndp = { x: box.x + r.x, y: box.y + r.y };
    L0.srcRect = `${nsr.x},${nsr.y},${nsr.w},${nsr.h}`;
    L0.dstPos = `${round(ndp.x)},${round(ndp.y)}`;
    clampLayerSrcRectToCaptureProxy(L0);
  } else if (cropMode === 'removeSplit') {
    commitCropRemoveSplit(L0, box, r);
  }

  ui.crop.rect = null;
  ui.crop.drag = null;
  ui.crop.active = false;
  setMode('select');
  validateCfg();
  renderAll();
}

function commitCropRemoveSplit(L0, box, r){
  const cropBoxVP = { x: box.x + r.x, y: box.y + r.y, w: r.w, h: r.h };

  const xA = box.x;
  const xC = cropBoxVP.x + cropBoxVP.w;
  const xD = box.x + box.w;

  const yB = cropBoxVP.y;
  const yC = cropBoxVP.y + cropBoxVP.h;
  const yD = box.y + box.h;

  const regions = [
    {nameSuffix:"TR", x:xC, y:yB, w:(xD-xC), h:(yC-yB)},
    {nameSuffix:"BL", x:xA, y:yC, w:(xC-xA), h:(yD-yC)},
    {nameSuffix:"BR", x:xC, y:yC, w:(xD-xC), h:(yD-yC)},
  ].filter(rr => rr.w >= 1 && rr.h >= 1);

  if (regions.length === 0) {
    cfg.layers.splice(selectedIdx,1);
    selectedIdx = -1;
    return;
  }

  const sr0 = parseRect(L0.srcRect);
  const sc0 = parseScale(L0.scale);
  const canMapSrc = (sr0 && sc0 && sr0.w>0 && sr0.h>0 && sc0.sx>0 && sc0.sy>0);

  function makeLayerForRegion(region){
    const L = deepClone(L0);
    L.name = (L0.name||'Layer') + "_" + region.nameSuffix;
    L.type = "video";

    if (canMapSrc) {
      const offXvp = (region.x - box.x);
      const offYvp = (region.y - box.y);
      const offXsrc = offXvp / sc0.sx;
      const offYsrc = offYvp / sc0.sy;
      const wSrc = Math.max(1, Math.round(region.w / sc0.sx));
      const hSrc = Math.max(1, Math.round(region.h / sc0.sy));

      L.srcRect = `${Math.round(sr0.x + offXsrc)},${Math.round(sr0.y + offYsrc)},${wSrc},${hSrc}`;
      L.scale = `${fmt(sc0.sx)},${fmt(sc0.sy)}`;
      L.dstPos = `${round(region.x)},${round(region.y)}`;
      clampLayerSrcRectToCaptureProxy(L);
    } else {
      L.dstPos = `${round(region.x)},${round(region.y)}`;
    }
    return ensureLayerDefaults(L);
  }

  const newLayers = regions.map(makeLayerForRegion);
  cfg.layers.splice(selectedIdx, 1, newLayers[0]);
  for (let i=1;i<newLayers.length;i++){
    cfg.layers.splice(selectedIdx+i, 0, newLayers[i]);
  }
}

/*** Rendering ***/
function renderStage(){
  updateStageMetrics();
  const stageEl = document.getElementById('overlayStage');
  stageEl.style.width = stage.stageW + "px";
  stageEl.style.height = stage.stageH + "px";
  stageEl.innerHTML = '';

  // build boxes (render only; pointer logic handled at stage level)
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
    lab.textContent = `${idx}: ${L.name||'Layer'}`;
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

    // crop UI within selected layer
    if (ui.mode === 'crop' && idx === selectedIdx) {
      const cropEl = document.createElement('div');
      cropEl.className = 'cropRect';
      cropEl.dataset.crop = '1';
      cropEl.dataset.layerIdx = String(idx);
      cropEl.style.display = 'none';

      const cap = document.createElement('div');
      cap.className = 'cap';
      cap.textContent = 'CROP';
      cropEl.appendChild(cap);

      ['nw','ne','sw','se'].forEach(h => {
        const ch = document.createElement('div');
        ch.className = 'ch c-' + h;
        ch.dataset.layerIdx = String(idx);
        ch.dataset.ch = h;
        cropEl.appendChild(ch);
      });

      // position crop rect
      if (ui.crop.rect) {
        const r = normalizeRect(ui.crop.rect, boxVP);
        ui.crop.rect = r;
        cropEl.style.display = 'block';
        cropEl.style.left = vpToStageX(boxVP.x + r.x) + 'px';
        cropEl.style.top  = vpToStageY(boxVP.y + r.y) + 'px';
        cropEl.style.width  = vpToStageX(r.w) + 'px';
        cropEl.style.height = vpToStageY(r.h) + 'px';
        updateCropReadout(boxVP);
      } else {
        document.getElementById('cropReadout').textContent = '(none)';
      }

      el.appendChild(cropEl);
    }

    stageEl.appendChild(el);
  });
}

function renderCrosshairPreview(){
  updateStageMetrics();
  const svg = document.getElementById('xhPreview');
  svg.setAttribute('width', stage.stageW);
  svg.setAttribute('height', stage.stageH);
  svg.setAttribute('viewBox', `0 0 ${stage.stageW} ${stage.stageH}`);
  svg.innerHTML = '';

  // If we have backend runtime mapping, use it. Otherwise fall back to old behavior.
  const haveRT = runtime && isFinite(runtime.outW) && isFinite(runtime.outH)
    && runtime.outW > 0 && runtime.outH > 0
    && runtime.presentRect && isFinite(runtime.presentRect.crtcW) && isFinite(runtime.presentRect.crtcH)
    && runtime.presentRect.crtcW > 0 && runtime.presentRect.crtcH > 0;

  // Backend considers crosshair center in OUTPUT pixels.
  const outW = haveRT ? runtime.outW : stage.vpw;
  const outH = haveRT ? runtime.outH : stage.vph;

  // We draw onto stage which is in "viewport coordinates" scaled to stage pixels.
  // stage.vpw/vph is the UI viewport coordinate space.
  // We need a function that maps an output pixel (ox,oy) -> viewport coordinate (vx,vy),
  // using backend presentRect (which maps viewport -> output).
  function outToVp(ox, oy){
    if (!haveRT) return {vx: ox, vy: oy}; // fallback, not correct when output != viewport

    const pr = runtime.presentRect;
    const vpw = (runtime.vpw && runtime.vpw > 0) ? runtime.vpw : stage.vpw;
    const vph = (runtime.vph && runtime.vph > 0) ? runtime.vph : stage.vph;

    // Invert:
    // ox = pr.crtcX + floor(vx * pr.crtcW / vpw)
    // approximate inverse by:
    // vx ~= (ox - pr.crtcX) * vpw / pr.crtcW
    // similarly for y
    const tx = ox - pr.crtcX;
    const ty = oy - pr.crtcY;

    const vx = (tx * vpw) / pr.crtcW;
    const vy = (ty * vph) / pr.crtcH;
    return {vx, vy};
  }

  cfg.layers.forEach((L, idx) => {
    ensureLayerDefaults(L);
    if (L.type !== 'crosshair') return;
    if (!L.enabled) return;
    if (!L.crosshair || !L.crosshair.enabled) return;

    const X = L.crosshair;
    const op = clamp(Number(X.opacity ?? 1), 0, 1);
    if (op <= 0) return;

    const d = parseDimWxH(X.diam);
    if (!d) return;

    let cx = outW/2, cy = outH/2;
    if (X.center && String(X.center).trim().length) {
      const p = parsePos(X.center);
      if (p) { cx = p.x; cy = p.y; }
    }

    const halfW = d.w/2;
    const halfH = d.h/2;
    const thick = clamp(Number(X.thickness ?? 1), 1, 99);

    // Map output-space center and extents into viewport coordinates
    const c0 = outToVp(cx, cy);
    const left  = outToVp(cx - halfW, cy).vx;
    const right = outToVp(cx + halfW, cy).vx;
    const top   = outToVp(cx, cy - halfH).vy;
    const bot   = outToVp(cx, cy + halfH).vy;

    const thx = Math.max(1, thick * (Math.abs(right-left) / Math.max(1, (2*halfW))));
    const thy = Math.max(1, thick * (Math.abs(bot-top)   / Math.max(1, (2*halfH))));

    // Convert viewport coords -> stage pixel coords
    const px = vpToStageX(c0.vx);
    const py = vpToStageY(c0.vy);
    const hw = Math.abs(vpToStageX(right) - vpToStageX(left)) / 2;
    const hh = Math.abs(vpToStageY(bot) - vpToStageY(top)) / 2;

    let color = 'rgba(255,0,0,0.95)';
    const rgb = parseRGB(X.color);
    if (rgb && X.mode !== 'invert') {
      color = `rgba(${clamp(round(rgb.r),0,255)},${clamp(round(rgb.g),0,255)},${clamp(round(rgb.b),0,255)},0.95)`;
    }
    if (X.mode === 'invert') color = 'rgba(255,209,102,0.95)';

    const outline = document.createElementNS('http://www.w3.org/2000/svg','rect');
    outline.setAttribute('x', (px - hw).toString());
    outline.setAttribute('y', (py - hh).toString());
    outline.setAttribute('width', (hw*2).toString());
    outline.setAttribute('height', (hh*2).toString());
    outline.setAttribute('fill', 'none');
    outline.setAttribute('stroke', 'rgba(255,209,102,0.65)');
    outline.setAttribute('stroke-width', '2');
    outline.setAttribute('stroke-dasharray', '6 6');
    outline.setAttribute('opacity', (op*0.6).toString());

    const hRect = document.createElementNS('http://www.w3.org/2000/svg','rect');
    hRect.setAttribute('x', (px - hw).toString());
    hRect.setAttribute('y', (py - (thy/2)).toString());
    hRect.setAttribute('width', (hw*2).toString());
    hRect.setAttribute('height', (thy).toString());
    hRect.setAttribute('fill', color);
    hRect.setAttribute('fill-opacity', (op*0.85).toString());

    const vRect = document.createElementNS('http://www.w3.org/2000/svg','rect');
    vRect.setAttribute('x', (px - (thx/2)).toString());
    vRect.setAttribute('y', (py - hh).toString());
    vRect.setAttribute('width', (thx).toString());
    vRect.setAttribute('height', (hh*2).toString());
    vRect.setAttribute('fill', color);
    vRect.setAttribute('fill-opacity', (op*0.85).toString());

    const label = document.createElementNS('http://www.w3.org/2000/svg','text');
    label.textContent = `${idx}:${L.name||'Crosshair'}`;
    label.setAttribute('x', (px + 8).toString());
    label.setAttribute('y', (py - 8).toString());
    label.setAttribute('fill', 'rgba(232,238,252,0.9)');
    label.setAttribute('font-size', '12');
    label.setAttribute('font-family', 'ui-monospace, Menlo, Consolas, monospace');

    svg.appendChild(outline);
    svg.appendChild(hRect);
    svg.appendChild(vRect);
    svg.appendChild(label);
  });
}

function renderLayersTable(){
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
        renderCrosshairPreview();
      };
    });

    tr.querySelectorAll('button').forEach(btn => {
      btn.onclick = () => {
        const a = btn.getAttribute('data-act');
        if (a==='sel'){ selectedIdx = idx; setMode('select'); renderAll(); }
        if (a==='del'){ cfg.layers.splice(idx,1); if(selectedIdx===idx) selectedIdx=-1; if(selectedIdx>idx) selectedIdx--; setMode('select'); renderAll(); }
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

function renderInspector(){
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
      ? 'Crosshair preview shown in viewport.'
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
    document.getElementById('iSrcRect').value = L.srcRect || '0,0,1,1';
    document.getElementById('iDstPos').value = L.dstPos || '0,0';
    document.getElementById('iScale').value = L.scale || '1.0,1.0';
    const sr = parseRect(L.srcRect);
    let msg='';
    if (!sr) msg = 'Invalid srcRect format x,y,w,h';
    else if (sr.w <= 0 || sr.h <= 0) msg = 'w/h must be >0';
    else msg = `Capture proxy bounds: ${stage.imgNaturalW||'?'}×${stage.imgNaturalH||'?'}`;
    document.getElementById('iSrcRectHelp').textContent = msg;
  }

  if (L.type === 'crosshair') {
    const X = L.crosshair;
    document.getElementById('iXEnabled').checked = !!X.enabled;
    document.getElementById('iXOpacity').value = clamp(Number(X.opacity ?? 1),0,1);
    document.getElementById('iXDiam').value = X.diam || '50x50';
    document.getElementById('iXThick').value = clamp(Number(X.thickness ?? 1), 1, 99);
    document.getElementById('iXMode').value = (X.mode === 'invert') ? 'invert' : 'solid';
    document.getElementById('iXColor').value = X.color || '255,255,255';
    document.getElementById('iXInvertRel').value = X.invertRel || 'none';
    const centerIsManual = !!(X.center && String(X.center).length);
    document.getElementById('iXCenterMode').value = centerIsManual ? 'manual' : 'auto';
    document.getElementById('iXCenter').value = X.center || '';
  }
}

function validateCfg(){
  const errors = [];

  if (cfg.viewport && cfg.viewport.trim().length) {
    const d = parseDimWxH(cfg.viewport);
    if (!d) errors.push('viewport: invalid (use WxH)');
  }

  cfg.layers.forEach((L, idx) => {
    ensureLayerDefaults(L);
    if (!['video','crosshair','graphics'].includes(L.type)) errors.push(`layer ${idx}: unknown type ${L.type}`);
    if (!['none','lower','upper'].includes(L.invertRel)) errors.push(`layer ${idx}: invalid invertRel`);
    const op = Number(L.opacity);
    if (!isFinite(op) || op < 0 || op > 1) errors.push(`layer ${idx}: opacity must be 0..1`);

    // IMPORTANT: apply_from_body currently requires these fields to be parseable even for crosshair layers
    if (!parseRect(L.srcRect)) errors.push(`layer ${idx}: srcRect invalid`);
    if (!parsePos(L.dstPos)) errors.push(`layer ${idx}: dstPos invalid`);
    if (!parseScale(L.scale)) errors.push(`layer ${idx}: scale invalid`);

    if (L.type === 'video') {
      const sr = parseRect(L.srcRect);
      const sc = parseScale(L.scale);
      if (!sr || sr.w<=0 || sr.h<=0) errors.push(`layer ${idx}: srcRect w/h must be >0`);
      if (!sc || !(sc.sx>0 && sc.sy>0)) errors.push(`layer ${idx}: scale must be >0`);
    }

    if (L.type === 'crosshair') {
      const X = L.crosshair || {};
      if (!['solid','invert'].includes(X.mode)) errors.push(`layer ${idx}: crosshair mode invalid`);
      if (!['none','lower','upper'].includes(X.invertRel)) errors.push(`layer ${idx}: crosshair invertRel invalid`);
      const d = parseDimWxH(X.diam);
      if (!d) errors.push(`layer ${idx}: crosshair diam invalid (WxH)`);
      const t = Number(X.thickness);
      if (!isFinite(t) || t<1 || t>99) errors.push(`layer ${idx}: crosshair thickness 1..99`);
      const xo = Number(X.opacity);
      if (!isFinite(xo) || xo<0 || xo>1) errors.push(`layer ${idx}: crosshair opacity 0..1`);
      if (X.color) {
        const c = parseRGB(X.color);
        if (!c) errors.push(`layer ${idx}: crosshair color invalid`);
      }
      if (X.center && String(X.center).length) {
        const p = parsePos(X.center);
        if (!p) errors.push(`layer ${idx}: crosshair center invalid`);
      }
    }
  });

  const box = document.getElementById('validationBox');
  if (errors.length === 0) {
    box.textContent = 'ok';
    box.className = 'help mono';
  } else {
    box.textContent = errors.map(e => '• ' + e).join('\n');
    box.className = 'err mono';
  }
  return errors;
}

/*** Status + load/apply ***/
async function refreshStatus(){
  const st = await api('/api/status');
  runtime = (st && st.runtime) ? st.runtime : null;
  document.getElementById('status').textContent = JSON.stringify(st, null, 2);
  // If runtime changes (reinit, mode change), refresh preview mapping.
  renderCrosshairPreview();
}

function setFormFromCfg(c){
  document.getElementById('v4l2Dev').value = c.v4l2Dev||'/dev/video0';
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
  // Before sending payload, hard-normalize layers to avoid backend apply failures.
  cfg.layers = (cfg.layers||[]).map(L => ensureLayerDefaults(L));
  return {
    v4l2Dev: document.getElementById('v4l2Dev').value,
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

/*** UI actions ***/
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
    cfg.layers = cfg.layers || [];
    const base = {
      name:'Video' + cfg.layers.length,
      type:'video',
      enabled:true,
      srcRect:'0,0,640,360',
      dstPos:'0,0',
      scale:'1.0,1.0',
      opacity:1.0,
      invertRel:'none',
      crosshair:{enabled:false,diam:'50x50',center:'',thickness:1,mode:'solid',color:'255,255,255',opacity:1.0,invertRel:'none'}
    };
    cfg.layers.push(ensureLayerDefaults(base));
    selectedIdx = cfg.layers.length-1;
    setMode('select');
    validateCfg();
    renderAll();
  };

  document.getElementById('btnAddCrosshair').onclick = () => {
    cfg.layers = cfg.layers || [];
    const base = {
      name:'Crosshair' + cfg.layers.length,
      type:'crosshair',
      enabled:true,
      opacity:1.0,
      invertRel:'none',
      // keep valid strings so backend apply doesn't reject
      srcRect:'0,0,1,1',
      dstPos:'0,0',
      scale:'1.0,1.0',
      crosshair:{enabled:true,diam:'120x120',center:'',thickness:2,mode:'solid',color:'255,0,0',opacity:1.0,invertRel:'none'}
    };
    cfg.layers.push(ensureLayerDefaults(base));
    selectedIdx = cfg.layers.length-1;
    setMode('select');
    validateCfg();
    renderAll();
  };

  document.getElementById('btnCrop').onclick = () => {
    if (selectedIdx < 0 || selectedIdx >= cfg.layers.length || cfg.layers[selectedIdx].type !== 'video') {
      alert("Select a video layer first.");
      return;
    }
    setMode('crop');
    ui.crop.rect = null;
    ui.crop.drag = null;
    ui.crop.active = false;
    renderAll();
  };
  document.getElementById('btnCropCancel').onclick = () => setMode('select');
  document.getElementById('btnCropCommit').onclick = () => commitCrop();

  // form hooks
  document.getElementById('viewport').oninput = () => { cfg.viewport = document.getElementById('viewport').value; renderVPHelp(); validateCfg(); renderStage(); renderCrosshairPreview(); };
  document.getElementById('present').onchange = () => { cfg.present = document.getElementById('present').value; validateCfg(); };
  document.getElementById('preferOutputMode').onchange = () => { cfg.preferOutputMode = document.getElementById('preferOutputMode').checked; };
  document.getElementById('preferOutputRes').onchange = () => { cfg.preferOutputRes = document.getElementById('preferOutputRes').value; };
  document.getElementById('v4l2Dev').onchange = () => { cfg.v4l2Dev = document.getElementById('v4l2Dev').value; };
  document.getElementById('edidPath').onchange = () => { cfg.edidPath = document.getElementById('edidPath').value; };
  document.getElementById('threads').onchange = () => { cfg.threads = Number(document.getElementById('threads').value); };
  document.getElementById('bgr').onchange = () => { cfg.bgr = document.getElementById('bgr').checked; };

  window.addEventListener('keydown', (ev) => {
    if (ui.mode === 'crop') {
      if (ev.key === 'Escape') { setMode('select'); ev.preventDefault(); }
      if (ev.key === 'Enter') { commitCrop(); ev.preventDefault(); }
    }
  });

  window.addEventListener('resize', () => renderAll());
}

function hookInspector(){
  function rerenderAll(){
    validateCfg();
    renderLayersTable();
    renderInspector();
    renderStage();
    renderCrosshairPreview();
  }
  function onChange(fn){ return () => { fn(); rerenderAll(); }; }

  document.getElementById('iName').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(L) L.name = document.getElementById('iName').value; });
  document.getElementById('iEnabled').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(L) L.enabled = document.getElementById('iEnabled').checked; });
  document.getElementById('iOpacity').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(L) L.opacity = clamp(Number(document.getElementById('iOpacity').value),0,1); });
  document.getElementById('iInvertRel').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(L) L.invertRel = document.getElementById('iInvertRel').value; });

  document.getElementById('iSrcRect').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(L) L.srcRect = document.getElementById('iSrcRect').value.trim(); });
  document.getElementById('iDstPos').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(L) L.dstPos = document.getElementById('iDstPos').value.trim(); });
  document.getElementById('iScale').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(L) L.scale = document.getElementById('iScale').value.trim(); });

  document.getElementById('btnClampSrc').onclick = () => {
    const L = cfg.layers[selectedIdx];
    if (!L || L.type !== 'video') return;
    clampLayerSrcRectToCaptureProxy(L);
    rerenderAll();
  };

  document.getElementById('iXEnabled').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L); L.crosshair.enabled = document.getElementById('iXEnabled').checked; });
  document.getElementById('iXOpacity').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L); L.crosshair.opacity = clamp(Number(document.getElementById('iXOpacity').value),0,1); });
  document.getElementById('iXDiam').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L); L.crosshair.diam = document.getElementById('iXDiam').value.trim(); });
  document.getElementById('iXThick').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L); L.crosshair.thickness = clamp(Number(document.getElementById('iXThick').value),1,99); });
  document.getElementById('iXMode').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L); L.crosshair.mode = document.getElementById('iXMode').value; });
  document.getElementById('iXColor').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L); L.crosshair.color = document.getElementById('iXColor').value.trim(); });
  document.getElementById('iXInvertRel').onchange = onChange(() => { const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L); L.crosshair.invertRel = document.getElementById('iXInvertRel').value; });
  document.getElementById('iXCenterMode').onchange = onChange(() => {
    const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L);
    const mode = document.getElementById('iXCenterMode').value;
    if (mode === 'auto') { L.crosshair.center = ""; document.getElementById('iXCenter').value = ""; }
    else { if (!L.crosshair.center) { L.crosshair.center = "0,0"; document.getElementById('iXCenter').value = "0,0"; } }
  });
  document.getElementById('iXCenter').onchange = onChange(() => {
    const L=cfg.layers[selectedIdx]; if(!L) return; ensureLayerDefaults(L);
    const mode = document.getElementById('iXCenterMode').value;
    if (mode === 'auto') { L.crosshair.center = ""; document.getElementById('iXCenter').value = ""; }
    else { L.crosshair.center = document.getElementById('iXCenter').value.trim(); }
  });

  document.getElementById('btnDupLayer').onclick = () => {
    if (selectedIdx < 0) return;
    const copy = deepClone(cfg.layers[selectedIdx]);
    copy.name = (copy.name||'Layer') + "_copy";
    cfg.layers.splice(selectedIdx+1, 0, copy);
    selectedIdx = selectedIdx+1;
    rerenderAll();
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
  renderLayersTable();
  renderInspector();
  renderStage();
  renderCrosshairPreview();
}

setInterval(refreshStatus, 1000);

(async () => {
  hookGlobalUI();
  hookInspector();
  hookImageNaturalSize();
  hookStagePointer();
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

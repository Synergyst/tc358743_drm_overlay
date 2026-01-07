#include "oak_accel.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

struct OakAccel::Impl {
  std::mutex mtx;

  std::unique_ptr<dai::Device> device;
  std::shared_ptr<dai::DataInputQueue> qVis;
  std::shared_ptr<dai::DataInputQueue> qThr;
  std::shared_ptr<dai::DataOutputQueue> qOut;

  std::atomic<bool> running{false};
  std::thread rxThread;

  int W = 0;
  int H = 0;

  // Latest output (GRAY8)
  std::mutex outMtx;
  std::vector<uint8_t> outGray;
  uint64_t outSeq = 0;
  std::atomic<bool> outReady{false};

  // Helper to create ImgFrame message
  static std::shared_ptr<dai::ImgFrame> makeGrayFrame(int w, int h, const uint8_t* data, size_t bytes, uint64_t seq) {
    auto f = std::make_shared<dai::ImgFrame>();
    f->setType(dai::ImgFrame::Type::GRAY8);
    f->setWidth(w);
    f->setHeight(h);
    f->setSequenceNum((int64_t)seq);
    f->setTimestamp(std::chrono::steady_clock::now());
    f->setData(std::vector<uint8_t>(data, data + bytes));
    return f;
  }

  void startRx() {
    running.store(true);
    rxThread = std::thread([this]() {
      while(running.load()) {
        try {
          // non-blocking check
          auto msg = qOut->tryGet<dai::ImgFrame>();
          if(!msg) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
          }
          auto data = msg->getData();
          if((int)data.size() != W * H) {
            continue;
          }
          {
            std::lock_guard<std::mutex> lk(outMtx);
            outGray = std::move(data);
            outSeq = (uint64_t)msg->getSequenceNum();
            outReady.store(true);
          }
        } catch(...) {
          // If device disconnects or queue errors, stop thread.
          running.store(false);
          break;
        }
      }
    });
  }

  void stopRx() {
    running.store(false);
    if(rxThread.joinable()) rxThread.join();
  }
};

OakAccel::OakAccel() : p(new Impl) {}
OakAccel::~OakAccel() { shutdown(); }

bool OakAccel::init(const std::string& deviceMxId, int width, int height, std::string* err) {
  if(width <= 0 || height <= 0) {
    if(err) *err = "invalid width/height";
    return false;
  }
  shutdown();

  W = width;
  H = height;
  p->W = width;
  p->H = height;

  try {
    dai::Pipeline pipeline;

    auto xinVis = pipeline.create<dai::node::XLinkIn>();
    auto xinThr = pipeline.create<dai::node::XLinkIn>();
    auto script = pipeline.create<dai::node::Script>();
    auto xout   = pipeline.create<dai::node::XLinkOut>();

    xinVis->setStreamName("oak_visible");
    xinThr->setStreamName("oak_thermal");
    xout->setStreamName("oak_out");

    // We send GRAY8 frames W*H
    xinVis->setMaxDataSize((uint32_t)(W * H));
    xinThr->setMaxDataSize((uint32_t)(W * H));

    // Script receives two inputs
    xinVis->out.link(script->inputs["vis"]);
    xinThr->out.link(script->inputs["thr"]);
    script->outputs["out"].link(xout->input);

    // Device-side processing:
    // - Wait until both a visible and thermal frame are available (latest).
    // - Compute thermal edge map using a simple Sobel-like approximation.
    // - Overlay edges onto visible (brighten where edge strong).
    //
    // NOTE: This is intentionally simple. We can improve later (warp/mesh/etc.).
    std::ostringstream ss;
    ss << R"PY(
import time

W = )PY" << W << R"PY(
H = )PY" << H << R"PY(

def sobel_mag(gray, W, H):
    # gray: bytearray length W*H
    out = bytearray(W*H)
    # Borders left as 0
    for y in range(1, H-1):
        row = y*W
        for x in range(1, W-1):
            i = row + x
            p00 = gray[i - W - 1]
            p01 = gray[i - W]
            p02 = gray[i - W + 1]
            p10 = gray[i - 1]
            p12 = gray[i + 1]
            p20 = gray[i + W - 1]
            p21 = gray[i + W]
            p22 = gray[i + W + 1]

            gx = (-p00 + p02) + (-2*p10 + 2*p12) + (-p20 + p22)
            gy = (-p00 -2*p01 -p02) + ( p20 +2*p21 + p22)
            mag = abs(gx) + abs(gy)
            if mag > 255: mag = 255
            out[i] = mag
    return out

def overlay_edges(vis, edges, W, H, threshold=32, boost=180):
    # vis: bytearray W*H
    # edges: bytearray W*H
    out = bytearray(W*H)
    for i in range(W*H):
        v = vis[i]
        e = edges[i]
        if e >= threshold:
            vv = v + boost
            if vv > 255: vv = 255
            out[i] = vv
        else:
            out[i] = v
    return out

latest_vis = None
latest_thr = None

while True:
    # non-blocking pull
    v = node.io['vis'].tryGet()
    if v is not None:
        latest_vis = v
    t = node.io['thr'].tryGet()
    if t is not None:
        latest_thr = t

    if latest_vis is None or latest_thr is None:
        time.sleep(0.001)
        continue

    visData = bytearray(latest_vis.getData())
    thrData = bytearray(latest_thr.getData())

    # thermal edges
    edges = sobel_mag(thrData, W, H)

    # overlay onto visible
    outData = overlay_edges(visData, edges, W, H)

    # Reuse the visible frame container for output (same W/H/GRAY8),
    # just overwrite its data buffer and push it out.
    latest_vis.setData(outData)
    node.io['out'].send(latest_vis)

    # consume only latest; then wait for new frames to avoid re-sending same result too fast
    latest_vis = None
    latest_thr = None
)PY";
    script->setScript(ss.str());

    // Device
    if(deviceMxId.empty()) {
      p->device = std::make_unique<dai::Device>(pipeline);
    } else {
      dai::DeviceInfo info(deviceMxId);
      p->device = std::make_unique<dai::Device>(pipeline, info);
    }

    // Queues
    p->qVis = p->device->getInputQueue("oak_visible");
    p->qThr = p->device->getInputQueue("oak_thermal");
    p->qOut = p->device->getOutputQueue("oak_out", 4, false);

    p->startRx();
    return true;

  } catch(const std::exception& e) {
    if(err) *err = std::string("DepthAI init failed: ") + e.what();
    shutdown();
    return false;
  }
}

bool OakAccel::isInitialized() const {
  return p && p->device && p->qVis && p->qThr && p->qOut;
}

bool OakAccel::sendVisibleGray(const uint8_t* gray, size_t bytes, uint64_t seq, std::string* err) {
  if(!isInitialized()) {
    if(err) *err = "OAK not initialized";
    return false;
  }
  if((int)bytes != W * H) {
    if(err) *err = "visible bytes != W*H";
    return false;
  }
  try {
    auto f = Impl::makeGrayFrame(W, H, gray, bytes, seq);
    p->qVis->send(f);
    return true;
  } catch(const std::exception& e) {
    if(err) *err = std::string("sendVisibleGray failed: ") + e.what();
    return false;
  }
}

bool OakAccel::sendThermalGray(const uint8_t* gray, size_t bytes, uint64_t seq, std::string* err) {
  if(!isInitialized()) {
    if(err) *err = "OAK not initialized";
    return false;
  }
  if((int)bytes != W * H) {
    if(err) *err = "thermal bytes != W*H";
    return false;
  }
  try {
    auto f = Impl::makeGrayFrame(W, H, gray, bytes, seq);
    p->qThr->send(f);
    return true;
  } catch(const std::exception& e) {
    if(err) *err = std::string("sendThermalGray failed: ") + e.what();
    return false;
  }
}

bool OakAccel::tryGetOutputGray(std::vector<uint8_t>& outGray, uint64_t* outSeq, std::string* err) {
  if(!isInitialized()) {
    if(err) *err = "OAK not initialized";
    return false;
  }
  if(!p->outReady.load()) return false;

  std::lock_guard<std::mutex> lk(p->outMtx);
  if(!p->outReady.load()) return false;
  outGray = p->outGray;
  if(outSeq) *outSeq = p->outSeq;
  p->outReady.store(false);
  if((int)outGray.size() != W * H) {
    if(err) *err = "output size mismatch";
    return false;
  }
  return true;
}

void OakAccel::shutdown() {
  if(!p) return;
  p->stopRx();
  p->qVis.reset();
  p->qThr.reset();
  p->qOut.reset();
  p->device.reset();
  W = 0;
  H = 0;
  p->W = 0;
  p->H = 0;
  p->outReady.store(false);
  {
    std::lock_guard<std::mutex> lk(p->outMtx);
    p->outGray.clear();
    p->outSeq = 0;
  }
}

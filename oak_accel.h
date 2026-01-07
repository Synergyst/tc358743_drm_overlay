#pragma once
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

// Simple host-side DepthAI helper for:
//  - sending a visible GRAY8 frame (W x H)
//  - sending a thermal  GRAY8 frame (W x H)
//  - receiving an output GRAY8 overlay frame (W x H)
//
// Notes:
//  - This is intentionally minimal and synchronous-friendly.
//  - Internally it uses non-blocking queues and keeps "latest" frames,
//    so your compositor can call tryGetOutput() without stalling.
//
// Output semantics for v0:
//  - output = visible with thermal edges overlayed (brightened).
//  - If only one of visible/thermal has been provided yet, output may be unavailable.
class OakAccel {
public:
  enum class Status {
    OK = 0,
    NOT_INITIALIZED = 1,
    NO_DEVICE = 2,
    PIPELINE_ERROR = 3,
  };

  OakAccel();
  ~OakAccel();

  OakAccel(const OakAccel&) = delete;
  OakAccel& operator=(const OakAccel&) = delete;

  // Initialize OAK pipeline.
  // If deviceMxId is empty, it connects to the first available device.
  // width/height refer to the small processing resolution (e.g. 320x180).
  bool init(const std::string& deviceMxId, int width, int height, std::string* err);

  bool isInitialized() const;

  int width() const { return W; }
  int height() const { return H; }

  // Send GRAY8 frame. Data must be exactly W*H bytes.
  // These calls are lightweight; they enqueue the latest frame.
  bool sendVisibleGray(const uint8_t* gray, size_t bytes, uint64_t seq, std::string* err);
  bool sendThermalGray(const uint8_t* gray, size_t bytes, uint64_t seq, std::string* err);

  // Try to receive output. Non-blocking.
  // Returns true and fills outGray if a new output frame is available.
  bool tryGetOutputGray(std::vector<uint8_t>& outGray, uint64_t* outSeq, std::string* err);

  // Best-effort shutdown.
  void shutdown();

private:
  struct Impl;
  std::unique_ptr<Impl> p;

  int W = 0;
  int H = 0;
};

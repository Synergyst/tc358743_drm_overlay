#include "httplib.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

// These are declared in your header as static currently; ideally they should be
// defined in ONE .cpp and declared extern in the header.
// For now, assume they are accessible in this TU (or include the header that has them).
extern std::mutex g_frame_mtx;
extern std::condition_variable g_frame_cv;
extern std::vector<uint8_t> g_current_mjpeg_frame;
extern uint64_t g_frame_sequence;

// Helper to add no-cache headers (good for live streams).
static inline void set_no_cache_headers(httplib::Response& res) {
  res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  res.set_header("Pragma", "no-cache");
  res.set_header("Expires", "0");
  // Helps some reverse proxies not buffer.
  res.set_header("X-Accel-Buffering", "no");
}

// Call this where you set up your routes (where `svr` is in scope).
void install_live_stream_endpoints(httplib::Server& svr) {
  // Embedded viewer page
  svr.Get("/live", [](const httplib::Request& req, httplib::Response& res) {
    (void)req;
    set_no_cache_headers(res);

    // Simple embedded HTML that displays the MJPEG stream.
    // You can enhance CSS as desired.
    const char* html = R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Live Stream</title>
  <style>
    body { margin: 0; font-family: sans-serif; background: #111; color: #eee; }
    header { padding: 12px 16px; background: #1b1b1b; border-bottom: 1px solid #333; }
    main { padding: 16px; display: grid; place-items: center; }
    img { max-width: 100%; height: auto; background: #000; border: 1px solid #333; }
    .hint { opacity: 0.8; margin-top: 8px; font-size: 12px; }
  </style>
</head>
<body>
  <header>
    <strong>Live MJPEG Stream</strong>
  </header>
  <main>
    <div>
      <img id="stream" src="/stream.mjpeg" alt="Live stream" />
      <div class="hint">
        If it stalls, reload this page. This uses <code>multipart/x-mixed-replace</code>.
      </div>
    </div>
  </main>
</body>
</html>)HTML";

    res.set_content(html, "text/html; charset=utf-8");
  });

  // Optional: redirect / -> /live
  svr.Get("/", [](const httplib::Request& req, httplib::Response& res) {
    (void)req;
    res.status = 302;
    res.set_header("Location", "/live");
  });

  // MJPEG stream
  svr.Get("/stream.mjpeg", [](const httplib::Request& req, httplib::Response& res) {
    (void)req;

    set_no_cache_headers(res);

    // Content-Type includes boundary token "frame"
    // Each part begins with: --frame
    res.set_content_provider(
      "multipart/x-mixed-replace; boundary=frame",
      [](size_t /*offset*/, httplib::DataSink& sink) -> bool {
        uint64_t last_sent_seq = 0;

        for (;;) {
          std::vector<uint8_t> frame_to_send;

          // Wait for a new frame or quit
          {
            std::unique_lock<std::mutex> lk(g_frame_mtx);
            g_frame_cv.wait(lk, [&] {
              return g_frame_sequence > last_sent_seq;
            });

            // Copy frame data out while holding the lock briefly
            frame_to_send = g_current_mjpeg_frame;
            last_sent_seq = g_frame_sequence;
          }

          if (frame_to_send.empty()) {
            // Avoid sending empty frames; just wait for next.
            continue;
          }

          // Build multipart chunk header
          std::string header;
          header.reserve(128);
          header += "--frame\r\n";
          header += "Content-Type: image/jpeg\r\n";
          header += "Content-Length: " + std::to_string(frame_to_send.size()) + "\r\n";
          header += "\r\n";

          // Write header + JPEG + trailing CRLF
          if (!sink.write(header.data(), header.size())) return false;

          if (!sink.write(reinterpret_cast<const char*>(frame_to_send.data()),
                          frame_to_send.size())) return false;

          if (!sink.write("\r\n", 2)) return false;

          // Client disconnected?
          if (sink.is_writable && !sink.is_writable()) return false;
        }

        // Unreachable
        // return true;
      }
    );
  });
}

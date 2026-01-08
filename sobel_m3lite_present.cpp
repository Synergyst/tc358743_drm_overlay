// sobel_m3lite_present.cpp
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <errno.h>

static void die(const char* msg) {
    std::fprintf(stderr, "fatal: %s (errno=%d: %s)\n", msg, errno, std::strerror(errno));
    std::exit(1);
}
static void eglDie(const char* where) {
    EGLint e = eglGetError();
    std::fprintf(stderr, "fatal: %s (eglGetError=0x%x)\n", where, e);
    std::exit(1);
}
static void glDie(const char* where) {
    GLenum e = glGetError();
    std::fprintf(stderr, "fatal: %s (glGetError=0x%x)\n", where, e);
    std::exit(1);
}

static GLuint compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096];
        GLsizei n = 0;
        glGetShaderInfoLog(s, sizeof(log), &n, log);
        std::fprintf(stderr, "shader compile error:\n%.*s\n", (int)n, log);
        std::exit(1);
    }
    return s;
}
static GLuint linkProgram(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glBindAttribLocation(p, 0, "aPos");
    glBindAttribLocation(p, 1, "aUV");
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        GLsizei n = 0;
        glGetProgramInfoLog(p, sizeof(log), &n, log);
        std::fprintf(stderr, "program link error:\n%.*s\n", (int)n, log);
        std::exit(1);
    }
    return p;
}

// ----- V4L2 capture (YUYV) -----
struct MMapBuf { void* ptr=nullptr; size_t len=0; };

static int xioctl(int fd, unsigned long req, void* arg) {
    for (;;) {
        int r = ioctl(fd, req, arg);
        if (r == 0) return 0;
        if (errno == EINTR) continue;
        return -1;
    }
}

static void extractLumaFromYUYV(
    const uint8_t* srcYUYV, int srcStride,
    uint8_t* dstY, int w, int h
) {
    for (int y = 0; y < h; y++) {
        const uint8_t* s = srcYUYV + y * srcStride;
        uint8_t* d = dstY + y * w;
        for (int x = 0; x < w; x += 2) {
            d[x + 0] = s[0];
            d[x + 1] = s[2];
            s += 4;
        }
    }
}

struct V4L2Capture {
    int fd = -1;
    int w = 0, h = 0;
    int strideBytes = 0;
    std::vector<MMapBuf> bufs;

    void openDevice(const char* dev, int width, int height) {
        w = width; h = height;
        fd = ::open(dev, O_RDWR | O_CLOEXEC);
        if (fd < 0) die("open v4l2 device failed");

        v4l2_capability caps{};
        if (xioctl(fd, VIDIOC_QUERYCAP, &caps) < 0) die("VIDIOC_QUERYCAP failed");
        if (!(caps.capabilities & V4L2_CAP_VIDEO_CAPTURE)) die("not a capture device");
        if (!(caps.capabilities & V4L2_CAP_STREAMING)) die("device doesn't support streaming");

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = w;
        fmt.fmt.pix.height = h;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) die("VIDIOC_S_FMT failed");

        // Use what driver actually gave us
        w = fmt.fmt.pix.width;
        h = fmt.fmt.pix.height;
        strideBytes = fmt.fmt.pix.bytesperline;

        v4l2_requestbuffers req{};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) die("VIDIOC_REQBUFS failed");
        if (req.count < 2) die("insufficient v4l2 buffers");

        bufs.resize(req.count);

        for (unsigned i = 0; i < req.count; i++) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index = i;
            if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) die("VIDIOC_QUERYBUF failed");

            void* p = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
            if (p == MAP_FAILED) die("mmap v4l2 buffer failed");
            bufs[i].ptr = p;
            bufs[i].len = b.length;
        }

        for (unsigned i = 0; i < bufs.size(); i++) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index = i;
            if (xioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF failed");
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) die("VIDIOC_STREAMON failed");
    }

    // Returns pointer/stride valid until you requeue.
    bool dequeue(const uint8_t** outPtr, int* outStride, unsigned* outIndex) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) return false;
        *outPtr = (const uint8_t*)bufs[b.index].ptr;
        *outStride = strideBytes;
        *outIndex = b.index;
        return true;
    }

    void requeue(unsigned idx) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = idx;
        if (xioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF (requeue) failed");
    }

    void closeDevice() {
        if (fd >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(fd, VIDIOC_STREAMOFF, &type);
        }
        for (auto& b : bufs) {
            if (b.ptr && b.ptr != MAP_FAILED) munmap(b.ptr, b.len);
            b.ptr = nullptr; b.len = 0;
        }
        bufs.clear();
        if (fd >= 0) ::close(fd);
        fd = -1;
    }

    ~V4L2Capture() { closeDevice(); }
};

// ----- FB present using bitfields (fixes diagonal / wrong 16bpp assumptions) -----
static inline uint32_t maskN(unsigned n) { return (n >= 32) ? 0xFFFFFFFFu : ((1u << n) - 1u); }

static uint32_t packToFB(uint8_t r, uint8_t g, uint8_t b, uint8_t a,
                         const fb_var_screeninfo& v) {
    // Scale 8-bit channels to target length
    auto scale = [](uint8_t c, unsigned len) -> uint32_t {
        if (len == 0) return 0;
        uint32_t maxv = maskN(len);
        // (c/255) * maxv with rounding
        return (uint32_t(c) * maxv + 127) / 255;
    };

    uint32_t pr = scale(r, v.red.length);
    uint32_t pg = scale(g, v.green.length);
    uint32_t pb = scale(b, v.blue.length);
    uint32_t pa = scale(a, v.transp.length);

    uint32_t out = 0;
    out |= (pr & maskN(v.red.length))     << v.red.offset;
    out |= (pg & maskN(v.green.length))   << v.green.offset;
    out |= (pb & maskN(v.blue.length))    << v.blue.offset;
    out |= (pa & maskN(v.transp.length))  << v.transp.offset;
    return out;
}

static void presentCanvasToFB(
    const uint32_t* canvasXRGB, int canvasW, int canvasH, int canvasStridePixels,
    uint8_t* fbMem, int fbStrideBytes,
    const fb_var_screeninfo& vinfo
) {
    const int fbBpp = (int)vinfo.bits_per_pixel;
    const int fbBytesPerPixel = fbBpp / 8;
    if (!(fbBpp == 16 || fbBpp == 24 || fbBpp == 32)) {
        std::fprintf(stderr, "fatal: unsupported fb bpp=%d\n", fbBpp);
        std::exit(1);
    }

    // Only draw within visible resolution
    const int drawW = std::min(canvasW, (int)vinfo.xres);
    const int drawH = std::min(canvasH, (int)vinfo.yres);

    for (int y = 0; y < drawH; y++) {
        const uint32_t* src = canvasXRGB + y * canvasStridePixels;
        uint8_t* dstRow = fbMem + y * fbStrideBytes;

        for (int x = 0; x < drawW; x++) {
            uint32_t px = src[x]; // 0xFFRRGGBB (we treat top byte as A=FF)
            uint8_t a = 0xFF;
            uint8_t r = (px >> 16) & 0xFF;
            uint8_t g = (px >> 8) & 0xFF;
            uint8_t b = (px >> 0) & 0xFF;

            uint32_t packed = packToFB(r, g, b, a, vinfo);

            uint8_t* d = dstRow + x * fbBytesPerPixel;
            // fbdev expects native endianness packing. On Pi (little-endian), store LSB first.
            if (fbBytesPerPixel == 2) {
                d[0] = (packed >> 0) & 0xFF;
                d[1] = (packed >> 8) & 0xFF;
            } else if (fbBytesPerPixel == 3) {
                d[0] = (packed >> 0) & 0xFF;
                d[1] = (packed >> 8) & 0xFF;
                d[2] = (packed >> 16) & 0xFF;
            } else { // 4
                d[0] = (packed >> 0) & 0xFF;
                d[1] = (packed >> 8) & 0xFF;
                d[2] = (packed >> 16) & 0xFF;
                d[3] = (packed >> 24) & 0xFF;
            }
        }
    }
}

// ----- GPU Sobel (right half) -----
class SobelOffload {
public:
    SobelOffload(int w, int h)
        : W(w), H(h), RX(w/2), RW(w/2), RH(h),
          roiRGBA(RW * RH * 4)
    {}

    void init() {
        auto eglGetPlatformDisplayEXT =
            (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
        if (!eglGetPlatformDisplayEXT) die("eglGetPlatformDisplayEXT not available");

        dpy = eglGetPlatformDisplayEXT(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
        if (dpy == EGL_NO_DISPLAY) eglDie("eglGetPlatformDisplayEXT(EGL_PLATFORM_SURFACELESS_MESA) failed");
        if (!eglInitialize(dpy, nullptr, nullptr)) eglDie("eglInitialize failed");

        const EGLint cfgAttribs[] = {
            EGL_SURFACE_TYPE, 0,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 0,
            EGL_NONE
        };
        EGLConfig cfg;
        EGLint num = 0;
        if (!eglChooseConfig(dpy, cfgAttribs, &cfg, 1, &num) || num < 1)
            eglDie("eglChooseConfig failed");

        const EGLint ctxAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
        ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctxAttribs);
        if (ctx == EGL_NO_CONTEXT) eglDie("eglCreateContext failed");

        if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx))
            eglDie("eglMakeCurrent(EGL_NO_SURFACE) failed");

        setupGL();
    }

    void processRightHalf(const uint8_t* inputY, uint8_t* outputRightHalfY) {
        glBindTexture(GL_TEXTURE_2D, inTex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_LUMINANCE, GL_UNSIGNED_BYTE, inputY);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, W, H);

        glUseProgram(prog);

        glClear(GL_COLOR_BUFFER_BIT);

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), quad + 0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), quad + 2);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glFinish();

        glReadPixels(RX, 0, RW, RH, GL_RGBA, GL_UNSIGNED_BYTE, roiRGBA.data());
        if (glGetError() != GL_NO_ERROR) glDie("glReadPixels");

        for (int y = 0; y < RH; y++) {
            int srcY = (RH - 1 - y);
            const uint8_t* src = roiRGBA.data() + (srcY * RW) * 4;
            uint8_t* dst = outputRightHalfY + y * RW;
            for (int x = 0; x < RW; x++) dst[x] = src[x * 4 + 0];
        }
    }

    ~SobelOffload() {
        if (dpy != EGL_NO_DISPLAY) {
            eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (prog) glDeleteProgram(prog);
            if (inTex) glDeleteTextures(1, &inTex);
            if (outTex) glDeleteTextures(1, &outTex);
            if (fbo) glDeleteFramebuffers(1, &fbo);
            if (ctx != EGL_NO_CONTEXT) eglDestroyContext(dpy, ctx);
            eglTerminate(dpy);
        }
    }

private:
    void setupGL() {
        glGenTextures(1, &outTex);
        glBindTexture(GL_TEXTURE_2D, outTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, outTex, 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            die("FBO not complete");

        const char* vsSrc = R"(
            attribute vec2 aPos;
            attribute vec2 aUV;
            varying vec2 vUV;
            void main() { vUV = aUV; gl_Position = vec4(aPos, 0.0, 1.0); }
        )";

        const char* fsSrc = R"(
            precision mediump float;
            uniform sampler2D uTex;
            uniform vec2 uTexel;
            uniform float uSplitX;
            varying vec2 vUV;

            float L(vec2 uv) { return texture2D(uTex, uv).r; }

            void main() {
                float src = L(vUV);

                float tl = L(vUV + uTexel * vec2(-1.0, -1.0));
                float  t = L(vUV + uTexel * vec2( 0.0, -1.0));
                float tr = L(vUV + uTexel * vec2( 1.0, -1.0));
                float l  = L(vUV + uTexel * vec2(-1.0,  0.0));
                float r  = L(vUV + uTexel * vec2( 1.0,  0.0));
                float bl = L(vUV + uTexel * vec2(-1.0,  1.0));
                float  b = L(vUV + uTexel * vec2( 0.0,  1.0));
                float br = L(vUV + uTexel * vec2( 1.0,  1.0));

                float gx = -tl - 2.0*l - bl + tr + 2.0*r + br;
                float gy = -tl - 2.0*t - tr + bl + 2.0*b + br;

                float mag = clamp(sqrt(gx*gx + gy*gy), 0.0, 1.0);

                float m = step(uSplitX, vUV.x);
                float outv = mix(src, mag, m);

                gl_FragColor = vec4(outv, outv, outv, 1.0);
            }
        )";

        GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
        GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
        prog = linkProgram(vs, fs);
        glDeleteShader(vs);
        glDeleteShader(fs);

        glUseProgram(prog);
        GLint locTex   = glGetUniformLocation(prog, "uTex");
        GLint locTexel = glGetUniformLocation(prog, "uTexel");
        GLint locSplit = glGetUniformLocation(prog, "uSplitX");
        glUniform1i(locTex, 0);
        glUniform2f(locTexel, 1.0f / float(W), 1.0f / float(H));
        glUniform1f(locSplit, 0.5f);

        glGenTextures(1, &inTex);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, inTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, W, H, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, nullptr);
    }

private:
    int W, H;
    int RX, RW, RH;

    EGLDisplay dpy = EGL_NO_DISPLAY;
    EGLContext ctx = EGL_NO_CONTEXT;

    GLuint prog = 0;
    GLuint inTex = 0;
    GLuint outTex = 0;
    GLuint fbo = 0;

    std::vector<uint8_t> roiRGBA;

    const GLfloat quad[16] = {
        -1.f, -1.f, 0.f, 1.f,
         1.f, -1.f, 1.f, 1.f,
        -1.f,  1.f, 0.f, 0.f,
         1.f,  1.f, 1.f, 0.f,
    };
};

static void blitGrayROIToXRGB8888Canvas(
    uint32_t* canvas, int canvasStridePixels,
    const uint8_t* roiGray,
    int dstX, int dstY,
    int roiW, int roiH
) {
    for (int y = 0; y < roiH; y++) {
        uint32_t* row = canvas + (dstY + y) * canvasStridePixels + dstX;
        const uint8_t* src = roiGray + y * roiW;
        for (int x = 0; x < roiW; x++) {
            uint8_t v = src[x];
            row[x] = 0xFF000000u | (uint32_t(v) << 16) | (uint32_t(v) << 8) | uint32_t(v);
        }
    }
}

int main() {
    const int capW = 1280, capH = 512;

    const int canvasW = 1280, canvasH = 720;
    const int canvasStridePixels = canvasW;

    // Map framebuffer
    int fbFd = ::open("/dev/fb0", O_RDWR);
    if (fbFd < 0) die("open(/dev/fb0) failed");

    fb_var_screeninfo vinfo{};
    fb_fix_screeninfo finfo{};
    if (ioctl(fbFd, FBIOGET_VSCREENINFO, &vinfo) < 0) die("FBIOGET_VSCREENINFO failed");
    if (ioctl(fbFd, FBIOGET_FSCREENINFO, &finfo) < 0) die("FBIOGET_FSCREENINFO failed");

    void* fbMap = mmap(nullptr, finfo.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fbFd, 0);
    if (fbMap == MAP_FAILED) die("mmap(/dev/fb0) failed");

    std::fprintf(stderr,
        "fb: %ux%u bpp=%u line_length=%u\n",
        vinfo.xres, vinfo.yres, vinfo.bits_per_pixel, finfo.line_length
    );
    std::fprintf(stderr,
        "fb bitfields: R(off=%u,len=%u) G(off=%u,len=%u) B(off=%u,len=%u) A(off=%u,len=%u)\n",
        vinfo.red.offset, vinfo.red.length,
        vinfo.green.offset, vinfo.green.length,
        vinfo.blue.offset, vinfo.blue.length,
        vinfo.transp.offset, vinfo.transp.length
    );

    // Open V4L2 capture from your M3Lite (/dev/video0 per your listing)
    V4L2Capture cap;
    cap.openDevice("/dev/video0", capW, capH);
    std::fprintf(stderr, "v4l2: %dx%d stride=%d pix=YUYV\n", cap.w, cap.h, cap.strideBytes);

    // GPU Sobel
    SobelOffload sobel(cap.w, cap.h);
    sobel.init();

    std::vector<uint8_t> luma(cap.w * cap.h);
    std::vector<uint8_t> sobelRight((cap.w / 2) * cap.h);

    // Internal canvas (XRGB8888)
    std::vector<uint32_t> canvas(canvasStridePixels * canvasH, 0xFF000000u); // black background

    // Capture one frame + display it (loop it if you want)
    const uint8_t* framePtr = nullptr;
    int frameStride = 0;
    unsigned idx = 0;
    if (!cap.dequeue(&framePtr, &frameStride, &idx)) die("VIDIOC_DQBUF failed");
    extractLumaFromYUYV(framePtr, frameStride, luma.data(), cap.w, cap.h);
    sobel.processRightHalf(luma.data(), sobelRight.data());
    cap.requeue(idx);

    // Composite Sobel to right half of top 512 lines
    blitGrayROIToXRGB8888Canvas(canvas.data(), canvasStridePixels,
                               sobelRight.data(),
                               cap.w / 2, 0,
                               cap.w / 2, cap.h);

    // Present to real framebuffer using its *actual* bitfields/stride
    presentCanvasToFB(canvas.data(), canvasW, canvasH, canvasStridePixels,
                      (uint8_t*)fbMap, finfo.line_length, vinfo);

    munmap(fbMap, finfo.smem_len);
    close(fbFd);

    // Keep capture alive long enough that you can see it before the program exits
    // (optional; remove if your environment doesn't need it)
    usleep(300000);

    return 0;
}

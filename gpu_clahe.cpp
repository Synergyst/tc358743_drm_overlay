#include "gpu_clahe.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static void logErr(const char* msg) {
    std::fprintf(stderr, "GpuCLAHE: %s\n", msg);
}

static bool checkGL(const char* where) {
    GLenum e = glGetError();
    if (e != GL_NO_ERROR) {
        std::fprintf(stderr, "GpuCLAHE GL error at %s: 0x%x\n", where, e);
        return false;
    }
    return true;
}

static GLuint compileCS(const char* src) {
    GLuint s = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);

    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096];
        GLsizei n = 0;
        glGetShaderInfoLog(s, sizeof(log), &n, log);
        std::fprintf(stderr, "GpuCLAHE compute shader compile error:\n%.*s\n", (int)n, log);
        glDeleteShader(s);
        return 0;
    }

    GLuint p = glCreateProgram();
    glAttachShader(p, s);
    glLinkProgram(p);
    glDeleteShader(s);

    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        GLsizei n = 0;
        glGetProgramInfoLog(p, sizeof(log), &n, log);
        std::fprintf(stderr, "GpuCLAHE program link error:\n%.*s\n", (int)n, log);
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

void GpuCLAHE::reset() {
    std::lock_guard<std::mutex> lock(mtx);

    ok = false;

    EGLDisplay edpy = (EGLDisplay)dpy;
    EGLContext ectx = (EGLContext)ctx;

    if (edpy && ectx) {
        eglMakeCurrent(edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ectx);

        if (progHist)      glDeleteProgram(progHist);
        if (progClip)      glDeleteProgram(progClip);
        if (progCdf)       glDeleteProgram(progCdf);
        if (progApply)     glDeleteProgram(progApply);
        if (progClearHist) glDeleteProgram(progClearHist);
        progHist = progClip = progCdf = progApply = progClearHist = 0;

        if (readFbo) glDeleteFramebuffers(1, &readFbo);
        readFbo = 0;

        if (texLuma) glDeleteTextures(1, &texLuma);
        if (texOut)  glDeleteTextures(1, &texOut);
        texLuma = texOut = 0;

        if (ssboHist) glDeleteBuffers(1, &ssboHist);
        if (ssboCdf)  glDeleteBuffers(1, &ssboCdf);
        ssboHist = ssboCdf = 0;

        eglDestroyContext(edpy, ectx);
        eglTerminate(edpy);
    }

    dpy = nullptr;
    ctx = nullptr;

    W = H = xTiles = yTiles = tileW = tileH = 0;
    outLuma.clear();
}

bool GpuCLAHE::init(int w, int h, int x_tiles, int y_tiles) {
    reset();

    W = w; H = h; xTiles = x_tiles; yTiles = y_tiles;
    if (W <= 0 || H <= 0) return false;
    if (xTiles < 2 || yTiles < 2) return false;
    if ((W % xTiles) != 0 || (H % yTiles) != 0) return false;

    tileW = W / xTiles;
    tileH = H / yTiles;

    auto eglGetPlatformDisplayEXT =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (!eglGetPlatformDisplayEXT) { logErr("eglGetPlatformDisplayEXT missing"); return false; }

    EGLDisplay edpy = eglGetPlatformDisplayEXT(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    if (edpy == EGL_NO_DISPLAY) { logErr("surfaceless display failed"); return false; }
    if (!eglInitialize(edpy, nullptr, nullptr)) { logErr("eglInitialize failed"); return false; }

    const EGLint cfgAttribs[] = {
        EGL_SURFACE_TYPE, 0,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };

    EGLConfig cfg;
    EGLint num = 0;
    if (!eglChooseConfig(edpy, cfgAttribs, &cfg, 1, &num) || num < 1) {
        logErr("eglChooseConfig failed (need GLES3)");
        return false;
    }

    const EGLint ctxAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
    EGLContext ectx = eglCreateContext(edpy, cfg, EGL_NO_CONTEXT, ctxAttribs);
    if (ectx == EGL_NO_CONTEXT) { logErr("eglCreateContext failed"); return false; }

    if (!eglMakeCurrent(edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ectx)) {
        logErr("eglMakeCurrent failed");
        return false;
    }

    GLint maxSSBO = 0;
    glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &maxSSBO);
    if (maxSSBO < 2) { logErr("SSBO not supported enough"); return false; }

    // Input luma texture: R8
    glGenTextures(1, &texLuma);
    glBindTexture(GL_TEXTURE_2D, texLuma);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8, W, H);

    // Output texture: RGBA8 (portable for imageStore)
    glGenTextures(1, &texOut);
    glBindTexture(GL_TEXTURE_2D, texOut);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, W, H);

    // Persistent FBO for readback
    glGenFramebuffers(1, &readFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, readFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texOut, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        logErr("readFbo incomplete");
        return false;
    }

    const int tileCount = xTiles * yTiles;
    const size_t histBytes = (size_t)tileCount * 256u * sizeof(uint32_t);

    glGenBuffers(1, &ssboHist);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboHist);
    glBufferData(GL_SHADER_STORAGE_BUFFER, histBytes, nullptr, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &ssboCdf);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboCdf);
    glBufferData(GL_SHADER_STORAGE_BUFFER, histBytes, nullptr, GL_DYNAMIC_DRAW);

    outLuma.resize((size_t)W * (size_t)H);

    const char* csClearHist = R"(
        #version 310 es
        layout(local_size_x=256, local_size_y=1) in;
        layout(std430, binding=0) buffer Hist { uint hist[]; };
        uniform uint uCount;
        void main() {
            uint i = gl_GlobalInvocationID.x;
            if (i >= uCount) return;
            hist[i] = 0u;
        }
    )";

    const char* csHist = R"(
        #version 310 es
        layout(local_size_x=16, local_size_y=16) in;
        layout(binding=0) uniform highp sampler2D uLuma;
        layout(std430, binding=0) buffer Hist { coherent uint hist[]; };

        uniform int uW, uH;
        uniform int uTileW, uTileH;
        uniform int uXTiles, uYTiles;

        void main() {
            ivec2 p = ivec2(gl_GlobalInvocationID.xy);
            if (p.x >= uW || p.y >= uH) return;

            int tx = p.x / uTileW;
            int ty = p.y / uTileH;
            int tileIndex = ty * uXTiles + tx;

            float lf = texelFetch(uLuma, p, 0).r;
            int L = int(clamp(lf * 255.0 + 0.5, 0.0, 255.0));

            uint idx = uint(tileIndex * 256 + L);
            atomicAdd(hist[idx], 1u);
        }
    )";

    // Clip + redistribute: one invocation per tile
    const char* csClip = R"(
        #version 310 es
        layout(local_size_x=1, local_size_y=1) in;
        layout(std430, binding=0) buffer Hist { uint hist[]; };

        uniform int uTileCount;
        uniform uint uClipLimit;

        void main() {
            uint tile = gl_GlobalInvocationID.x;
            if (tile >= uint(uTileCount)) return;

            uint base = tile * 256u;

            uint excess = 0u;
            for (uint i = 0u; i < 256u; i++) {
                uint v = hist[base + i];
                if (v > uClipLimit) {
                    excess += (v - uClipLimit);
                    hist[base + i] = uClipLimit;
                }
            }

            uint addv = excess / 256u;
            uint rem  = excess - addv * 256u;

            if (addv != 0u) {
                for (uint i = 0u; i < 256u; i++) hist[base + i] += addv;
            }
            for (uint i = 0u; i < rem; i++) hist[base + i] += 1u;
        }
    )";

    // CDF: one invocation per tile
    const char* csCdf = R"(
        #version 310 es
        layout(local_size_x=1, local_size_y=1) in;

        layout(std430, binding=0) buffer Hist { readonly uint hist[]; };
        layout(std430, binding=1) buffer Cdf  { writeonly uint cdf[]; };

        uniform int uTileCount;

        void main() {
            uint tile = gl_GlobalInvocationID.x;
            if (tile >= uint(uTileCount)) return;

            uint base = tile * 256u;
            uint sum = 0u;
            for (uint i = 0u; i < 256u; i++) {
                sum += hist[base + i];
                cdf[base + i] = sum;
            }
        }
    )";

    // Apply: output is RGBA8 (portable); grayscale replicated into RGB
    const char* csApply = R"(
        #version 310 es
        layout(local_size_x=16, local_size_y=16) in;

        layout(binding=0) uniform highp sampler2D uLuma;
        layout(rgba8, binding=0) writeonly uniform highp image2D uOut;

        layout(std430, binding=1) buffer Cdf { readonly uint cdf[]; };

        uniform int uW, uH;
        uniform int uTileW, uTileH;
        uniform int uXTiles, uYTiles;
        uniform uint uTilePix;

        float mapFromTile(int tileIndex, int L) {
            uint v = cdf[uint(tileIndex * 256 + L)];
            return float(v) / float(uTilePix);
        }

        void main() {
            ivec2 p = ivec2(gl_GlobalInvocationID.xy);
            if (p.x >= uW || p.y >= uH) return;

            float lf = texelFetch(uLuma, p, 0).r;
            int L = int(clamp(lf * 255.0 + 0.5, 0.0, 255.0));

            float fx = float(p.x) / float(uTileW) - 0.5;
            float fy = float(p.y) / float(uTileH) - 0.5;

            int tx0 = int(floor(fx));
            int ty0 = int(floor(fy));
            float wx = fract(fx);
            float wy = fract(fy);

            tx0 = clamp(tx0, 0, uXTiles - 1);
            ty0 = clamp(ty0, 0, uYTiles - 1);
            int tx1 = clamp(tx0 + 1, 0, uXTiles - 1);
            int ty1 = clamp(ty0 + 1, 0, uYTiles - 1);

            int t00 = ty0 * uXTiles + tx0;
            int t10 = ty0 * uXTiles + tx1;
            int t01 = ty1 * uXTiles + tx0;
            int t11 = ty1 * uXTiles + tx1;

            float m00 = mapFromTile(t00, L);
            float m10 = mapFromTile(t10, L);
            float m01 = mapFromTile(t01, L);
            float m11 = mapFromTile(t11, L);

            float mx0 = mix(m00, m10, wx);
            float mx1 = mix(m01, m11, wx);
            float m = mix(mx0, mx1, wy);

            imageStore(uOut, p, vec4(m, m, m, 1.0));
        }
    )";

    progClearHist = compileCS(csClearHist);
    progHist      = compileCS(csHist);
    progClip      = compileCS(csClip);
    progCdf       = compileCS(csCdf);
    progApply     = compileCS(csApply);

    if (!progClearHist || !progHist || !progClip || !progCdf || !progApply) {
        logErr("compileCS failed (need GLES 3.1 compute support)");
        return false;
    }

    dpy = (void*)edpy;
    ctx = (void*)ectx;
    ok = true;
    return true;
}

bool GpuCLAHE::run(const uint8_t* inLuma, float clip_limit, uint8_t* outLuma8) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!ok) return false;

    EGLDisplay edpy = (EGLDisplay)dpy;
    EGLContext ectx = (EGLContext)ctx;
    if (!eglMakeCurrent(edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ectx)) return false;

    const int tileCount = xTiles * yTiles;
    const uint32_t tilePix = (uint32_t)(tileW * tileH);

    // Upload luma
    glBindTexture(GL_TEXTURE_2D, texLuma);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_RED, GL_UNSIGNED_BYTE, inLuma);
    if (!checkGL("upload luma")) return false;

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssboHist);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssboCdf);

    // Clear hist
    glUseProgram(progClearHist);
    const uint32_t histCount = (uint32_t)tileCount * 256u;
    glUniform1ui(glGetUniformLocation(progClearHist, "uCount"), histCount);
    glDispatchCompute((GLuint)((histCount + 255u) / 256u), 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Histogram
    glUseProgram(progHist);
    glUniform1i(glGetUniformLocation(progHist, "uW"), W);
    glUniform1i(glGetUniformLocation(progHist, "uH"), H);
    glUniform1i(glGetUniformLocation(progHist, "uTileW"), tileW);
    glUniform1i(glGetUniformLocation(progHist, "uTileH"), tileH);
    glUniform1i(glGetUniformLocation(progHist, "uXTiles"), xTiles);
    glUniform1i(glGetUniformLocation(progHist, "uYTiles"), yTiles);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texLuma);
    glDispatchCompute((GLuint)((W + 15) / 16), (GLuint)((H + 15) / 16), 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    uint32_t clipBin = (uint32_t)(clip_limit * (float(tilePix) / 256.0f));
    if (clipBin < 1) clipBin = 1;

    // Clip
    glUseProgram(progClip);
    glUniform1i(glGetUniformLocation(progClip, "uTileCount"), tileCount);
    glUniform1ui(glGetUniformLocation(progClip, "uClipLimit"), clipBin);
    glDispatchCompute((GLuint)tileCount, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // CDF
    glUseProgram(progCdf);
    glUniform1i(glGetUniformLocation(progCdf, "uTileCount"), tileCount);
    glDispatchCompute((GLuint)tileCount, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Apply
    glUseProgram(progApply);
    glUniform1i(glGetUniformLocation(progApply, "uW"), W);
    glUniform1i(glGetUniformLocation(progApply, "uH"), H);
    glUniform1i(glGetUniformLocation(progApply, "uTileW"), tileW);
    glUniform1i(glGetUniformLocation(progApply, "uTileH"), tileH);
    glUniform1i(glGetUniformLocation(progApply, "uXTiles"), xTiles);
    glUniform1i(glGetUniformLocation(progApply, "uYTiles"), yTiles);
    glUniform1ui(glGetUniformLocation(progApply, "uTilePix"), tilePix);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texLuma);
    glBindImageTexture(0, texOut, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);

    glDispatchCompute((GLuint)((W + 15) / 16), (GLuint)((H + 15) / 16), 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_FRAMEBUFFER_BARRIER_BIT);

    // IMPORTANT: sync BEFORE readback (this is the fix for the glitch you described)
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
    glFinish();

    // Read back RGBA and take R
    glBindFramebuffer(GL_FRAMEBUFFER, readFbo);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) return false;

    std::vector<uint8_t> rgba((size_t)W * (size_t)H * 4);
    glBindFramebuffer(GL_FRAMEBUFFER, readFbo);
    glReadBuffer(GL_COLOR_ATTACHMENT0); // GLES 3.x supports this
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    if (!checkGL("glReadPixels")) return false;

    for (int y = 0; y < H; y++) {
        //int srcY = (H - 1 - y);
        int srcY = y;
        const uint8_t* src = rgba.data() + (size_t)srcY * (size_t)W * 4;
        uint8_t* dst = outLuma8 + (size_t)y * (size_t)W;
        for (int x = 0; x < W; x++) dst[x] = src[(size_t)x * 4 + 0];
    }
    return true;
}

GpuCLAHE::~GpuCLAHE() {
    reset();
}

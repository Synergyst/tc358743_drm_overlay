#include <turbojpeg.h>
#include <vector>
#include <cstdint>

bool encode_frame_to_jpeg(void* src_map, int w, int h, int pitch, std::vector<uint8_t>& out_jpeg) {
    if (!src_map) return false;

    // Initialize with the modern 3.x function
    tjhandle compressor = tj3Init(TJINIT_COMPRESS);
    if (!compressor) return false;

    // These parameters are valid in libturbojpeg 3.0.3
    //tj3Set(compressor, TJPARAM_QUALITY, 9);
    //tj3Set(compressor, TJPARAM_QUALITY, 18);
    //tj3Set(compressor, TJPARAM_QUALITY, 68);
    tj3Set(compressor, TJPARAM_QUALITY, 100);
    tj3Set(compressor, TJPARAM_SUBSAMP, TJSAMP_420);

    unsigned char* dest_buf = nullptr;
    size_t dest_size = 0; // Note: TurboJPEG 3 uses size_t for sizes

    int status = tj3Compress8(
        compressor,
        static_cast<const unsigned char*>(src_map),
        w, pitch, h,
        TJPF_BGRX,
        &dest_buf, &dest_size
    );

    if (status == 0) {
        out_jpeg.assign(dest_buf, dest_buf + dest_size);
    }

    tj3Free(dest_buf);
    tj3Destroy(compressor);
    return (status == 0);
}

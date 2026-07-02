// Lossless compression of RAW 32FC1 (single-channel float32) depth images.
//
// Design: a small registry of "methods". Each method is a pair of pure
// functions that transform a raw float image (width*height floats) to/from a
// compressed byte blob. ZSTD-on-raw-floats is the baseline; new approaches
// (byte-plane split, spatial predictors, etc.) register alongside it and the
// benchmark compares them all on equal footing.
//
// "level" is a per-method tuning knob (for zstd it is the compression level).
//
// The public encode_depth()/decode_depth() wrap a chosen method in a
// self-describing blob so a decoder needs nothing but the bytes.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace depthcodec {

struct Method {
  std::string name;
  // Encode width*height floats -> compressed bytes.
  std::vector<uint8_t> (*encode)(const float* data, uint32_t w, uint32_t h, int level);
  // Decode compressed bytes -> width*height floats (out must be preallocated).
  void (*decode)(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int level);
};

// Registry of all available methods.
const std::vector<Method>& methods();
const Method* find_method(const std::string& name);

// --- Public self-describing API (the deliverable) -------------------------
// Blob layout (little-endian):
//   magic 'D','P','C','1' | uint8 name_len | name bytes | int32 level
//   | uint32 width | uint32 height | payload...
std::vector<uint8_t> encode_depth(const float* data, uint32_t width, uint32_t height,
                                  const std::string& method = "dpred", int level = 1);

std::vector<float> decode_depth(const uint8_t* blob, size_t size,
                                uint32_t* out_width = nullptr, uint32_t* out_height = nullptr);

}  // namespace depthcodec

// Round-trip unit tests: every registered method must reproduce the input
// BIT-EXACTLY (memcmp over the raw float bytes) on images designed to hit
// the edge cases — NaN payloads, +/-0, +/-inf, denormals, the alp sentinel
// values (-1, -2, -3), high-cardinality dictionary overflow, and degenerate
// shapes. No test framework: CHECK() counts failures, main() reports.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "depth_codec/depth_codec.hpp"

namespace {

int g_failures = 0;

#define CHECK(cond, msg)                                              \
  do {                                                                \
    if (!(cond)) {                                                    \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, msg);       \
      ++g_failures;                                                   \
    }                                                                 \
  } while (0)

float from_bits(uint32_t u) {
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}

// Round-trip an image through the public API with one method and verify
// bit-exactness and recovered dimensions.
void roundtrip(const std::vector<float>& img, uint32_t w, uint32_t h,
               const std::string& method, int level, const char* what) {
  const std::vector<uint8_t> blob =
      depthcodec::encode_depth(img.data(), w, h, method, level);
  uint32_t ow = 0, oh = 0;
  const std::vector<float> back = depthcodec::decode_depth(blob.data(), blob.size(), &ow, &oh);
  const std::string ctx = std::string(what) + " [" + method + ":" + std::to_string(level) + "]";
  CHECK(ow == w && oh == h, (ctx + ": dimensions").c_str());
  CHECK(back.size() == img.size(), (ctx + ": size").c_str());
  if (back.size() == img.size() && !img.empty()) {
    CHECK(std::memcmp(back.data(), img.data(), img.size() * 4) == 0,
          (ctx + ": bit-exact payload").c_str());
  }
}

void roundtrip_all(const std::vector<float>& img, uint32_t w, uint32_t h, const char* what) {
  for (const auto& m : depthcodec::methods())
    for (int level : {1, 3}) roundtrip(img, w, h, m.name, level, what);
}

// Realistic depth frame: smooth gradient, quantized values, a NaN hole.
std::vector<float> make_depth(uint32_t w, uint32_t h) {
  std::vector<float> img(size_t(w) * h);
  for (uint32_t y = 0; y < h; ++y)
    for (uint32_t x = 0; x < w; ++x) {
      const int q = 1 + int((x * 131 + y * 17) % 5000);
      img[size_t(y) * w + x] = 10100.0f / (float(q) + 1009.0f);
    }
  for (uint32_t y = h / 4; y < h / 2; ++y)
    for (uint32_t x = w / 3; x < 2 * w / 3; ++x)
      img[size_t(y) * w + x] = from_bits(0x7FC00000u);  // canonical NaN hole
  return img;
}

// Every special bit pattern that has bitten a float codec at least once.
std::vector<float> make_specials(uint32_t& w, uint32_t& h) {
  const uint32_t bits[] = {
      0x00000000u,  // +0.0
      0x80000000u,  // -0.0
      0x7F800000u,  // +inf
      0xFF800000u,  // -inf
      0x7FC00000u,  // canonical quiet NaN
      0x7FC00001u,  // NaN, non-canonical payload
      0x7F800001u,  // signalling NaN
      0xFFC00000u,  // negative NaN
      0xFFFFFFFFu,  // NaN, all ones
      0x00000001u,  // smallest positive denormal
      0x807FFFFFu,  // negative denormal
      0x7F7FFFFFu,  // FLT_MAX
      0xFF7FFFFFu,  // -FLT_MAX
      0x00800000u,  // FLT_MIN
      0xBF800000u,  // -1.0 (alp NaN sentinel collision)
      0xC0000000u,  // -2.0 (alp +inf sentinel collision)
      0xC0400000u,  // -3.0 (alp -inf sentinel collision)
      0x3F800000u,  // 1.0
  };
  w = 6;
  h = 6;
  std::vector<float> img(size_t(w) * h);
  for (size_t i = 0; i < img.size(); ++i) img[i] = from_bits(bits[i % std::size(bits)]);
  return img;
}

}  // namespace

int main() {
  // 1. Realistic depth frame.
  roundtrip_all(make_depth(97, 53), 97, 53, "depth frame");

  // 2. Special values (non-finite, denormals, sentinel collisions).
  {
    uint32_t w, h;
    const std::vector<float> img = make_specials(w, h);
    roundtrip_all(img, w, h, "special values");
  }

  // 3. High cardinality: > 65536 distinct patterns forces the dict/dpred
  //    fallback path (and floods alp/alprd with exceptions).
  {
    std::mt19937 rng(12345);
    const uint32_t w = 512, h = 256;
    std::vector<float> img(size_t(w) * h);
    for (auto& f : img) f = from_bits(rng());
    roundtrip_all(img, w, h, "random bits");
  }

  // 4. Constant image (single dictionary entry, all-zero residuals).
  roundtrip_all(std::vector<float>(64 * 32, 1.25f), 64, 32, "constant");

  // 5. Degenerate shapes.
  roundtrip_all({0.5f}, 1, 1, "1x1");
  roundtrip_all({1.f, 2.f, 3.f, 4.f, 5.f}, 5, 1, "5x1");
  roundtrip_all({1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f}, 1, 7, "1x7");
  {
    const float dummy = 0.0f;
    const auto blob = depthcodec::encode_depth(&dummy, 0, 0, "dpred", 1);
    uint32_t ow = 1, oh = 1;
    const auto back = depthcodec::decode_depth(blob.data(), blob.size(), &ow, &oh);
    CHECK(ow == 0 && oh == 0 && back.empty(), "0x0 image");
  }

  // 6. Malformed input must throw, not crash or return garbage.
  {
    bool threw = false;
    try {
      const uint8_t junk[8] = {'X', 'X', 'X', 'X', 0, 0, 0, 0};
      depthcodec::decode_depth(junk, sizeof(junk));
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw, "bad magic throws");

    threw = false;
    try {
      const float one = 1.0f;
      depthcodec::encode_depth(&one, 1, 1, "no_such_method", 1);
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw, "unknown method throws");
  }

  if (g_failures == 0) {
    std::printf("all round-trip tests passed (%zu methods x 2 levels)\n",
                depthcodec::methods().size());
    return 0;
  }
  std::printf("%d FAILURES\n", g_failures);
  return 1;
}

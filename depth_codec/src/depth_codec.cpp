#include "depth_codec/depth_codec.hpp"

#include <zstd.h>

#if defined(__AVX2__)
#include <immintrin.h>
#endif

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace depthcodec {

namespace {

[[noreturn]] void fail(const char* what) { throw std::runtime_error(what); }

// ---- shared stage-2: ZSTD over an intermediate byte buffer ----------------
// Contexts are reused per thread: ZSTD_compress/ZSTD_decompress allocate and
// tear down a workspace on every call, which costs ~1 ms per frame.
struct ZstdCtx {
  ZSTD_CCtx* c = ZSTD_createCCtx();
  ZSTD_DCtx* d = ZSTD_createDCtx();
  ~ZstdCtx() {
    ZSTD_freeCCtx(c);
    ZSTD_freeDCtx(d);
  }
};

ZstdCtx& tl_zstd() {
  static thread_local ZstdCtx ctx;
  return ctx;
}

std::vector<uint8_t> zstd_pack(const uint8_t* src, size_t n, int level) {
  const size_t bound = ZSTD_compressBound(n);
  std::vector<uint8_t> out(bound);
  const size_t k =
      ZSTD_compressCCtx(tl_zstd().c, out.data(), bound, src, n, level <= 0 ? 3 : level);
  if (ZSTD_isError(k)) fail(ZSTD_getErrorName(k));
  out.resize(k);
  return out;
}

std::vector<uint8_t> zstd_unpack(const uint8_t* comp, size_t comp_size) {
  const unsigned long long sz = ZSTD_getFrameContentSize(comp, comp_size);
  if (sz == ZSTD_CONTENTSIZE_ERROR || sz == ZSTD_CONTENTSIZE_UNKNOWN) fail("bad zstd frame");
  std::vector<uint8_t> out(static_cast<size_t>(sz));
  const size_t k = ZSTD_decompressDCtx(tl_zstd().d, out.data(), out.size(), comp, comp_size);
  if (ZSTD_isError(k)) fail(ZSTD_getErrorName(k));
  if (k != out.size()) fail("zstd_unpack: size mismatch");
  return out;
}

// ---- little bitstream helpers (LSB-first) ---------------------------------
struct BitWriter {
  std::vector<uint8_t>& out;
  uint64_t acc = 0;
  int nbits = 0;
  void put(uint64_t v, int b) {
    while (b > 0) {
      const int take = std::min(b, 32);
      acc |= (v & ((1ull << take) - 1)) << nbits;
      nbits += take;
      v >>= take;
      b -= take;
      while (nbits >= 8) {
        out.push_back(static_cast<uint8_t>(acc));
        acc >>= 8;
        nbits -= 8;
      }
    }
  }
  void flush() {
    while (nbits > 0) {
      out.push_back(static_cast<uint8_t>(acc));
      acc >>= 8;
      nbits -= 8;
    }
    acc = 0;
    nbits = 0;
  }
};

struct BitReader {
  const uint8_t* p;
  size_t size;
  size_t pos = 0;
  uint64_t acc = 0;
  int nbits = 0;
  uint64_t get(int b) {
    uint64_t v = 0;
    int got = 0;
    while (got < b) {
      const int take = std::min(b - got, 32);
      while (nbits < take) {
        acc |= static_cast<uint64_t>(pos < size ? p[pos] : 0) << nbits;
        ++pos;
        nbits += 8;
      }
      v |= (acc & ((1ull << take) - 1)) << got;
      acc >>= take;
      nbits -= take;
      got += take;
    }
    return v;
  }
};

constexpr size_t kVec = 1024;  // ALP-style vector granularity

inline void put_u16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x));
  v.push_back(static_cast<uint8_t>(x >> 8));
}
inline void put_u32(std::vector<uint8_t>& v, uint32_t x) {
  for (int i = 0; i < 4; ++i) v.push_back(static_cast<uint8_t>(x >> (8 * i)));
}
inline void put_u64(std::vector<uint8_t>& v, uint64_t x) {
  for (int i = 0; i < 8; ++i) v.push_back(static_cast<uint8_t>(x >> (8 * i)));
}
inline uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t get_u32(const uint8_t* p) {
  uint32_t x = 0;
  for (int i = 0; i < 4; ++i) x |= static_cast<uint32_t>(p[i]) << (8 * i);
  return x;
}
inline uint64_t get_u64(const uint8_t* p) {
  uint64_t x = 0;
  for (int i = 0; i < 8; ++i) x |= static_cast<uint64_t>(p[i]) << (8 * i);
  return x;
}

// ---- Baseline: plain ZSTD over the raw float bytes ------------------------
std::vector<uint8_t> zstd_encode(const float* data, uint32_t w, uint32_t h, int level) {
  const size_t src_size = static_cast<size_t>(w) * h * sizeof(float);
  const size_t bound = ZSTD_compressBound(src_size);
  std::vector<uint8_t> out(bound);
  const size_t n = ZSTD_compress(out.data(), bound, data, src_size, level);
  if (ZSTD_isError(n)) fail(ZSTD_getErrorName(n));
  out.resize(n);
  return out;
}

void zstd_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t dst_size = static_cast<size_t>(w) * h * sizeof(float);
  const size_t n = ZSTD_decompress(out, dst_size, comp, comp_size);
  if (ZSTD_isError(n)) fail(ZSTD_getErrorName(n));
  if (n != dst_size) fail("zstd_decode: size mismatch");
}

// ---- Reference ceiling: no compression (memcpy) --------------------------
std::vector<uint8_t> store_encode(const float* data, uint32_t w, uint32_t h, int /*level*/) {
  const size_t src_size = static_cast<size_t>(w) * h * sizeof(float);
  std::vector<uint8_t> out(src_size);
  std::memcpy(out.data(), data, src_size);
  return out;
}

void store_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t dst_size = static_cast<size_t>(w) * h * sizeof(float);
  if (comp_size != dst_size) fail("store_decode: size mismatch");
  std::memcpy(out, comp, dst_size);
}

// ---- BSS: byte-stream split (transpose the 4 bytes of each float) + ZSTD --
// Groups byte 0 of every float together, then byte 1, etc. Each plane is far
// more self-similar than the interleaved stream, which helps ZSTD's entropy
// stage. Pure bit-domain: NaN/Inf/-0 round-trip untouched.
std::vector<uint8_t> bss_encode(const float* data, uint32_t w, uint32_t h, int level) {
  const size_t n = static_cast<size_t>(w) * h;
  const uint8_t* src = reinterpret_cast<const uint8_t*>(data);
  std::vector<uint8_t> planes(n * 4);
  for (size_t i = 0; i < n; ++i) {
    planes[i] = src[4 * i];
    planes[n + i] = src[4 * i + 1];
    planes[2 * n + i] = src[4 * i + 2];
    planes[3 * n + i] = src[4 * i + 3];
  }
  return zstd_pack(planes.data(), planes.size(), level);
}

void bss_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint8_t> planes = zstd_unpack(comp, comp_size);
  if (planes.size() != n * 4) fail("bss_decode: size mismatch");
  uint8_t* dst = reinterpret_cast<uint8_t*>(out);
  for (size_t i = 0; i < n; ++i) {
    dst[4 * i] = planes[i];
    dst[4 * i + 1] = planes[n + i];
    dst[4 * i + 2] = planes[2 * n + i];
    dst[4 * i + 3] = planes[3 * n + i];
  }
}

// ---- ALP (classic, decimal branch) + ZSTD ---------------------------------
// Per 1024-value vector: find a decimal exponent e so that
//   n = llround(v * 10^e)  reconstructs  v == float(double(n) / 10^e)
// bit-exactly, then frame-of-reference + bit-pack the integers. Values that
// don't round-trip are stored verbatim as (pos, raw word) exceptions.
//
// Value-domain transform, so non-finite values need the sentinel mapping:
//   canonical NaN (0x7FC00000) -> -1.0, +inf -> -2.0, -inf -> -3.0
// (depth is positive, so negatives are free). A real -1.0/-2.0/-3.0 in the
// input, or a NaN with a non-canonical payload, is forced into the exception
// list — mapping alone would be silently lossy for those.
constexpr uint32_t kCanonNaN = 0x7FC00000u;
constexpr uint32_t kPosInf = 0x7F800000u;
constexpr uint32_t kNegInf = 0xFF800000u;
constexpr double kPow10[11] = {1e0, 1e1, 1e2, 1e3, 1e4, 1e5,
                               1e6, 1e7, 1e8, 1e9, 1e10};

inline bool alp_exact(float v, int e, int64_t* out_n) {
  const double scaled = static_cast<double>(v) * kPow10[e];
  if (!(std::fabs(scaled) < 9.0e18)) return false;  // llround would overflow
  const int64_t n = std::llround(scaled);
  const float rec = static_cast<float>(static_cast<double>(n) / kPow10[e]);
  if (std::bit_cast<uint32_t>(rec) != std::bit_cast<uint32_t>(v)) return false;
  *out_n = n;
  return true;
}

std::vector<uint8_t> alp_encode(const float* data, uint32_t w, uint32_t h, int level) {
  const size_t n = static_cast<size_t>(w) * h;
  const uint32_t* words = reinterpret_cast<const uint32_t*>(data);
  std::vector<uint8_t> plain;
  plain.reserve(n * 4 / 2);

  std::vector<float> vf(kVec);
  std::vector<uint8_t> forced(kVec);
  std::vector<int64_t> ints(kVec);
  std::vector<uint8_t> is_exc(kVec);
  std::vector<std::pair<uint16_t, uint32_t>> excs;

  for (size_t base = 0; base < n; base += kVec) {
    const size_t cnt = std::min(kVec, n - base);

    // Sentinel-map into the value domain; flag collisions for the exc list.
    for (size_t i = 0; i < cnt; ++i) {
      const uint32_t wd = words[base + i];
      const bool is_nan = (wd & 0x7F800000u) == 0x7F800000u && (wd & 0x007FFFFFu) != 0;
      float v = std::bit_cast<float>(wd);
      bool force = false;
      if (wd == kCanonNaN) v = -1.0f;
      else if (wd == kPosInf) v = -2.0f;
      else if (wd == kNegInf) v = -3.0f;
      else if (is_nan || v == -1.0f || v == -2.0f || v == -3.0f) force = true;
      vf[i] = v;
      forced[i] = force;
    }

    // Pick e on a small sample: most hits wins, smaller e (smaller ints) ties.
    int best_e = 8, best_hits = -1;
    const size_t stride = std::max<size_t>(1, cnt / 32);
    for (int e = 0; e <= 10; ++e) {
      int hits = 0;
      int64_t dummy;
      for (size_t i = 0; i < cnt; i += stride)
        if (!forced[i] && alp_exact(vf[i], e, &dummy)) ++hits;
      if (hits > best_hits) { best_hits = hits; best_e = e; }
    }

    // Full pass: integers + exceptions, then frame-of-reference.
    excs.clear();
    int64_t mn = 0, mx = 0;
    bool have = false;
    for (size_t i = 0; i < cnt; ++i) {
      int64_t v_int = 0;
      if (forced[i] || !alp_exact(vf[i], best_e, &v_int)) {
        is_exc[i] = 1;
        excs.push_back({static_cast<uint16_t>(i), words[base + i]});
        continue;
      }
      is_exc[i] = 0;
      ints[i] = v_int;
      if (!have) { mn = mx = v_int; have = true; }
      else { mn = std::min(mn, v_int); mx = std::max(mx, v_int); }
    }
    const int b = have ? std::bit_width(static_cast<uint64_t>(mx - mn)) : 0;

    plain.push_back(static_cast<uint8_t>(best_e));
    plain.push_back(static_cast<uint8_t>(b));
    put_u16(plain, static_cast<uint16_t>(excs.size()));
    put_u64(plain, static_cast<uint64_t>(mn));
    BitWriter bw{plain};
    for (size_t i = 0; i < cnt; ++i)
      bw.put(is_exc[i] ? 0 : static_cast<uint64_t>(ints[i] - mn), b);
    bw.flush();
    for (const auto& e : excs) {
      put_u16(plain, e.first);
      put_u32(plain, e.second);
    }
  }
  return zstd_pack(plain.data(), plain.size(), level);
}

void alp_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint8_t> plain = zstd_unpack(comp, comp_size);
  uint32_t* words = reinterpret_cast<uint32_t*>(out);
  size_t pos = 0;
  const auto need = [&](size_t k) { if (pos + k > plain.size()) fail("alp_decode: truncated"); };

  for (size_t base = 0; base < n; base += kVec) {
    const size_t cnt = std::min(kVec, n - base);
    need(12);
    const int e = plain[pos];
    const int b = plain[pos + 1];
    const size_t excn = get_u16(&plain[pos + 2]);
    const int64_t mn = static_cast<int64_t>(get_u64(&plain[pos + 4]));
    pos += 12;
    if (e > 10 || b > 64) fail("alp_decode: bad header");

    const size_t stream_bytes = (cnt * static_cast<size_t>(b) + 7) / 8;
    need(stream_bytes);
    BitReader br{plain.data() + pos, stream_bytes};
    pos += stream_bytes;
    for (size_t i = 0; i < cnt; ++i) {
      const int64_t v_int = mn + static_cast<int64_t>(br.get(b));
      const float v = static_cast<float>(static_cast<double>(v_int) / kPow10[e]);
      uint32_t wd;
      if (v == -1.0f) wd = kCanonNaN;
      else if (v == -2.0f) wd = kPosInf;
      else if (v == -3.0f) wd = kNegInf;
      else wd = std::bit_cast<uint32_t>(v);
      words[base + i] = wd;
    }
    need(excn * 6);
    for (size_t k = 0; k < excn; ++k) {
      const uint16_t p16 = get_u16(&plain[pos]);
      const uint32_t wd = get_u32(&plain[pos + 2]);
      pos += 6;
      if (p16 >= cnt) fail("alp_decode: bad exception pos");
      words[base + p16] = wd;
    }
  }
}

// ---- ALP_RD (the "real doubles" branch, adapted to float32) + ZSTD --------
// Bit-domain: split each 32-bit word at bit r into left (high 32-r bits) and
// right (low r bits). Left parts are very low-cardinality for physical data,
// so each 1024-vector keeps a <=8-entry dictionary of its most frequent lefts
// (3-bit codes); lefts outside the dictionary go to an exception list.
// r >= 16 keeps every left in a uint16. No sentinels needed: NaN/Inf are just
// bit patterns.
int alprd_pick_r(const uint32_t* words, size_t n) {
  const size_t stride = std::max<size_t>(1, n / 4096);
  std::vector<uint16_t> lefts;
  lefts.reserve(n / stride + 1);
  double best_cost = 1e30;
  int best_r = 16;
  for (int r = 16; r <= 28; ++r) {
    lefts.clear();
    for (size_t i = 0; i < n; i += stride) lefts.push_back(static_cast<uint16_t>(words[i] >> r));
    std::sort(lefts.begin(), lefts.end());
    std::vector<uint32_t> counts;
    for (size_t i = 0; i < lefts.size();) {
      size_t j = i;
      while (j < lefts.size() && lefts[j] == lefts[i]) ++j;
      counts.push_back(static_cast<uint32_t>(j - i));
      i = j;
    }
    std::sort(counts.begin(), counts.end(), std::greater<>());
    size_t cov = 0;
    for (size_t k = 0; k < counts.size() && k < 8; ++k) cov += counts[k];
    const double exc_frac = lefts.empty() ? 0.0 : 1.0 - double(cov) / double(lefts.size());
    const double cost = 3.0 + r + exc_frac * 32.0;  // bits/value estimate
    if (cost < best_cost) { best_cost = cost; best_r = r; }
  }
  return best_r;
}

std::vector<uint8_t> alprd_encode(const float* data, uint32_t w, uint32_t h, int level) {
  const size_t n = static_cast<size_t>(w) * h;
  const uint32_t* words = reinterpret_cast<const uint32_t*>(data);
  std::vector<uint8_t> plain;
  plain.reserve(n * 4 / 2);
  const int r = n ? alprd_pick_r(words, n) : 16;
  plain.push_back(static_cast<uint8_t>(r));
  const uint32_t rmask = (1u << r) - 1;

  std::vector<uint16_t> lefts(kVec), sorted;
  std::vector<uint16_t> dict;
  std::vector<std::pair<uint16_t, uint16_t>> excs;

  for (size_t base = 0; base < n; base += kVec) {
    const size_t cnt = std::min(kVec, n - base);
    lefts.resize(cnt);
    for (size_t i = 0; i < cnt; ++i) lefts[i] = static_cast<uint16_t>(words[base + i] >> r);

    // Per-vector dictionary: the 8 most frequent left values.
    sorted = lefts;
    std::sort(sorted.begin(), sorted.end());
    struct Run { uint16_t v; uint32_t c; };
    std::vector<Run> runs;
    for (size_t i = 0; i < cnt;) {
      size_t j = i;
      while (j < cnt && sorted[j] == sorted[i]) ++j;
      runs.push_back({sorted[i], static_cast<uint32_t>(j - i)});
      i = j;
    }
    std::sort(runs.begin(), runs.end(), [](const Run& a, const Run& b) { return a.c > b.c; });
    dict.clear();
    for (size_t k = 0; k < runs.size() && k < 8; ++k) dict.push_back(runs[k].v);

    plain.push_back(static_cast<uint8_t>(dict.size()));
    const size_t excn_pos = plain.size();
    put_u16(plain, 0);  // patched below
    for (uint16_t dv : dict) put_u16(plain, dv);

    excs.clear();
    BitWriter codes{plain};
    for (size_t i = 0; i < cnt; ++i) {
      int code = -1;
      for (size_t k = 0; k < dict.size(); ++k)
        if (dict[k] == lefts[i]) { code = static_cast<int>(k); break; }
      if (code < 0) {
        excs.push_back({static_cast<uint16_t>(i), lefts[i]});
        code = 0;
      }
      codes.put(static_cast<uint64_t>(code), 3);
    }
    codes.flush();
    BitWriter rights{plain};
    for (size_t i = 0; i < cnt; ++i) rights.put(words[base + i] & rmask, r);
    rights.flush();

    plain[excn_pos] = static_cast<uint8_t>(excs.size());
    plain[excn_pos + 1] = static_cast<uint8_t>(excs.size() >> 8);
    for (const auto& e : excs) {
      put_u16(plain, e.first);
      put_u16(plain, e.second);
    }
  }
  return zstd_pack(plain.data(), plain.size(), level);
}

void alprd_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint8_t> plain = zstd_unpack(comp, comp_size);
  uint32_t* words = reinterpret_cast<uint32_t*>(out);
  size_t pos = 0;
  const auto need = [&](size_t k) { if (pos + k > plain.size()) fail("alprd_decode: truncated"); };

  need(1);
  const int r = plain[pos++];
  if (r < 16 || r > 28) fail("alprd_decode: bad r");

  uint16_t dict[8] = {0};
  std::vector<uint8_t> codes(kVec);
  std::vector<uint32_t> rights(kVec);

  for (size_t base = 0; base < n; base += kVec) {
    const size_t cnt = std::min(kVec, n - base);
    need(3);
    const size_t dn = plain[pos];
    const size_t excn = get_u16(&plain[pos + 1]);
    pos += 3;
    if (dn > 8) fail("alprd_decode: bad dict size");
    need(dn * 2);
    for (size_t k = 0; k < dn; ++k) { dict[k] = get_u16(&plain[pos]); pos += 2; }

    const size_t code_bytes = (cnt * 3 + 7) / 8;
    need(code_bytes);
    {
      BitReader br{plain.data() + pos, code_bytes};
      for (size_t i = 0; i < cnt; ++i) codes[i] = static_cast<uint8_t>(br.get(3));
    }
    pos += code_bytes;
    const size_t right_bytes = (cnt * static_cast<size_t>(r) + 7) / 8;
    need(right_bytes);
    {
      BitReader br{plain.data() + pos, right_bytes};
      for (size_t i = 0; i < cnt; ++i) rights[i] = static_cast<uint32_t>(br.get(r));
    }
    pos += right_bytes;

    for (size_t i = 0; i < cnt; ++i) {
      const uint16_t left = dict[codes[i] < dn ? codes[i] : 0];
      words[base + i] = (static_cast<uint32_t>(left) << r) | rights[i];
    }
    need(excn * 4);
    for (size_t k = 0; k < excn; ++k) {
      const uint16_t p16 = get_u16(&plain[pos]);
      const uint16_t left = get_u16(&plain[pos + 2]);
      pos += 4;
      if (p16 >= cnt) fail("alprd_decode: bad exception pos");
      words[base + p16] = (static_cast<uint32_t>(left) << r) | rights[p16];
    }
  }
}

// ---- shared: per-image value dictionary, bucketized + SIMD-probed ---------
// Maps each 32-bit pattern to a first-seen id. 8 keys per cache-line bucket,
// all compared at once; a per-bucket count masks stale slots. The found-
// branch is ~99% predictable (only first occurrences miss), which keeps the
// pipeline from flushing — a scalar probe loop was 3.5x slower here purely
// on branch mispredicts. Returns false if > 65536 distinct patterns.
struct alignas(64) DictBucket {  // exactly one cache line
  uint32_t keys[8];
  uint16_t vals[8];
  uint16_t cnt;
  uint16_t pad[7];
};
static_assert(sizeof(DictBucket) == 64);

bool build_value_dict(const uint32_t* words, size_t n, std::vector<uint32_t>& entries,
                      std::vector<uint16_t>& idx) {
  constexpr size_t nb = 1u << 14;  // 16384 buckets x 8 slots = 1 MB, > 2x max load
  constexpr size_t kMaxDict = 1u << 16;
  std::vector<DictBucket> table(nb);  // zero-init: all counts start at 0
  entries.clear();
  entries.reserve(1u << 12);
  idx.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const uint32_t k32 = words[i];
    size_t b = (k32 * 2654435761u) >> 18;  // top 14 bits
    uint32_t id;
    for (;;) {
      DictBucket& B = table[b];
      uint32_t m;
#if defined(__AVX2__)
      const __m256i vk = _mm256_set1_epi32(static_cast<int32_t>(k32));
      const __m256i keys = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(B.keys));
      m = static_cast<uint32_t>(
          _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpeq_epi32(keys, vk))));
      m &= (1u << B.cnt) - 1;
#else
      m = 0;
      for (unsigned k = 0; k < B.cnt; ++k)
        if (B.keys[k] == k32) { m = 1u << k; break; }
#endif
      if (m) {
        id = B.vals[std::countr_zero(m)];
        break;
      }
      if (B.cnt < 8) {
        if (entries.size() >= kMaxDict) return false;
        id = static_cast<uint32_t>(entries.size());
        B.keys[B.cnt] = k32;
        B.vals[B.cnt] = static_cast<uint16_t>(id);
        ++B.cnt;
        entries.push_back(k32);
        break;
      }
      b = (b + 1) & (nb - 1);  // bucket full: spill to the next one
    }
    idx[i] = static_cast<uint16_t>(id);
  }
  return true;
}

// ---- DICT: whole-value dictionary + index plane + ZSTD --------------------
// The r=0 degenerate of ALP_RD: if an image has <=65536 distinct 32-bit
// patterns, store the dictionary once and each pixel as a byte-aligned
// uint8/uint16 index (byte alignment keeps ZSTD's match-finding effective).
// Falls back to plain ZSTD when cardinality is too high, so it stays a
// general-purpose codec. Bit-domain: NaN-safe for free.
std::vector<uint8_t> dict_encode(const float* data, uint32_t w, uint32_t h, int level) {
  const size_t n = static_cast<size_t>(w) * h;
  const uint32_t* words = reinterpret_cast<const uint32_t*>(data);
  std::vector<uint32_t> entries;
  std::vector<uint16_t> idx;
  const bool overflow = !build_value_dict(words, n, entries, idx);

  std::vector<uint8_t> plain;
  if (overflow) {
    plain.resize(1 + n * 4);
    plain[0] = 0;
    std::memcpy(plain.data() + 1, words, n * 4);
  } else {
    const uint8_t idx_bytes = entries.size() <= 256 ? 1 : 2;
    plain.reserve(6 + entries.size() * 4 + n * idx_bytes);
    plain.push_back(1);
    plain.push_back(idx_bytes);
    put_u32(plain, static_cast<uint32_t>(entries.size()));
    for (uint32_t v : entries) put_u32(plain, v);
    if (idx_bytes == 1) {
      for (size_t i = 0; i < n; ++i) plain.push_back(static_cast<uint8_t>(idx[i]));
    } else {
      for (size_t i = 0; i < n; ++i) put_u16(plain, static_cast<uint16_t>(idx[i]));
    }
  }
  return zstd_pack(plain.data(), plain.size(), level);
}

void dict_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint8_t> plain = zstd_unpack(comp, comp_size);
  if (plain.empty()) fail("dict_decode: empty payload");
  uint32_t* words = reinterpret_cast<uint32_t*>(out);
  if (plain[0] == 0) {
    if (plain.size() != 1 + n * 4) fail("dict_decode: size mismatch");
    std::memcpy(words, plain.data() + 1, n * 4);
    return;
  }
  if (plain.size() < 6) fail("dict_decode: truncated header");
  const uint8_t idx_bytes = plain[1];
  const size_t ds = get_u32(&plain[2]);
  const size_t hdr = 6 + ds * 4;
  if (idx_bytes != 1 && idx_bytes != 2) fail("dict_decode: bad idx width");
  if (plain.size() != hdr + n * idx_bytes) fail("dict_decode: size mismatch");
  const uint8_t* dict = plain.data() + 6;
  const uint8_t* ip = plain.data() + hdr;
  for (size_t i = 0; i < n; ++i) {
    const size_t k = idx_bytes == 1 ? ip[i] : get_u16(ip + 2 * i);
    if (k >= ds) fail("dict_decode: bad index");
    words[i] = get_u32(dict + 4 * k);
  }
}

// ---- 2D prediction helpers -------------------------------------------------
// JPEG-LS / LOCO-I median-edge-detector predictor.
inline uint32_t med_predict(uint32_t a /*left*/, uint32_t b /*up*/, uint32_t c /*upleft*/) {
  const uint32_t mx = std::max(a, b), mn = std::min(a, b);
  if (c >= mx) return mn;
  if (c <= mn) return mx;
  return a + b - c;  // gradient; mod-2^32 wrap is fine, decode mirrors it
}

inline uint32_t zigzag32(uint32_t d) {  // d = wrapped difference
  const int32_t s = static_cast<int32_t>(d);
  return (static_cast<uint32_t>(s) << 1) ^ static_cast<uint32_t>(s >> 31);
}
inline uint32_t unzigzag32(uint32_t z) { return (z >> 1) ^ (~(z & 1) + 1); }

// Total-order-preserving bijection: unsigned comparison of the mapped word
// equals IEEE-754 total order of the float (negatives fixed up, NaN at the
// top). Keeps MED's min/max meaningful in the bit domain.
inline uint32_t float_to_ord(uint32_t w) { return (w >> 31) ? ~w : (w | 0x80000000u); }
inline uint32_t ord_to_float(uint32_t m) { return (m >> 31) ? (m ^ 0x80000000u) : ~m; }

// ---- FPRED: 2D MED prediction directly on float bit patterns + ZSTD -------
// Dict-free and cardinality-unlimited: map every word to its total-order
// integer, MED-predict from the left/up/up-left neighbours, zigzag the
// wrapped residual and split it into 4 byte planes for ZSTD. Smooth depth
// makes residuals tiny, so the high planes collapse to near-zero runs.
std::vector<uint8_t> fpred_encode(const float* data, uint32_t w, uint32_t h, int level) {
  const size_t n = static_cast<size_t>(w) * h;
  const uint32_t* words = reinterpret_cast<const uint32_t*>(data);
  std::vector<uint32_t> ord(n);
  for (size_t i = 0; i < n; ++i) ord[i] = float_to_ord(words[i]);

  std::vector<uint8_t> planes(n * 4);
  for (uint32_t y = 0; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    for (uint32_t x = 0; x < w; ++x) {
      const size_t i = row + x;
      uint32_t pred;
      if (y == 0) pred = x ? ord[i - 1] : 0;
      else if (x == 0) pred = ord[i - w];
      else pred = med_predict(ord[i - 1], ord[i - w], ord[i - w - 1]);
      const uint32_t z = zigzag32(ord[i] - pred);
      planes[i] = static_cast<uint8_t>(z);
      planes[n + i] = static_cast<uint8_t>(z >> 8);
      planes[2 * n + i] = static_cast<uint8_t>(z >> 16);
      planes[3 * n + i] = static_cast<uint8_t>(z >> 24);
    }
  }
  return zstd_pack(planes.data(), planes.size(), level);
}

void fpred_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint8_t> planes = zstd_unpack(comp, comp_size);
  if (planes.size() != n * 4) fail("fpred_decode: size mismatch");
  uint32_t* words = reinterpret_cast<uint32_t*>(out);
  std::vector<uint32_t> ord(n);
  for (uint32_t y = 0; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    for (uint32_t x = 0; x < w; ++x) {
      const size_t i = row + x;
      uint32_t pred;
      if (y == 0) pred = x ? ord[i - 1] : 0;
      else if (x == 0) pred = ord[i - w];
      else pred = med_predict(ord[i - 1], ord[i - w], ord[i - w - 1]);
      const uint32_t z = planes[i] | (planes[n + i] << 8) |
                         (static_cast<uint32_t>(planes[2 * n + i]) << 16) |
                         (static_cast<uint32_t>(planes[3 * n + i]) << 24);
      ord[i] = pred + unzigzag32(z);
      words[i] = ord_to_float(ord[i]);
    }
  }
}

// ---- DPRED: value-sorted dictionary + 2D MED prediction on indices --------
// Like `dict`, but the dictionary is sorted by float total order so the index
// is monotone in depth; then the uint16 index plane is MED-predicted in 2D
// and the zigzag residuals are stored as low/high byte planes. Falls back to
// plain ZSTD above 65536 distinct patterns, like `dict`.
std::vector<uint8_t> dpred_encode(const float* data, uint32_t w, uint32_t h, int level) {
  const size_t n = static_cast<size_t>(w) * h;
  const uint32_t* words = reinterpret_cast<const uint32_t*>(data);

  // Pass 1: dictionary + first-seen indices (shared bucketized builder).
  std::vector<uint32_t> entries;
  std::vector<uint16_t> idx;
  if (!build_value_dict(words, n, entries, idx)) {
    std::vector<uint8_t> plain(1 + n * 4);
    plain[0] = 0;
    std::memcpy(plain.data() + 1, words, n * 4);
    return zstd_pack(plain.data(), plain.size(), level);
  }

  // Sort the dictionary by float total order and remap indices so that
  // index distance ~ depth distance (what makes MED residuals small).
  // LSD radix sort (two 16-bit digits) over key = ord<<16 | original_index:
  // a comparison sort here was 40% of total encode time (up to 65k entries,
  // gather-heavy comparator, ~one branch mispredict per comparison).
  const size_t ds = entries.size();
  std::vector<uint64_t> keys_a(ds), keys_b(ds);
  for (size_t k = 0; k < ds; ++k)
    keys_a[k] = (static_cast<uint64_t>(float_to_ord(entries[k])) << 16) | k;
  {
    std::vector<uint32_t> hist(1u << 16);
    uint64_t* src = keys_a.data();
    uint64_t* dst = keys_b.data();
    for (const int shift : {16, 32}) {
      std::fill(hist.begin(), hist.end(), 0);
      for (size_t k = 0; k < ds; ++k) ++hist[(src[k] >> shift) & 0xFFFF];
      uint32_t sum = 0;
      for (uint32_t& slot : hist) { const uint32_t c = slot; slot = sum; sum += c; }
      for (size_t k = 0; k < ds; ++k) dst[hist[(src[k] >> shift) & 0xFFFF]++] = src[k];
      std::swap(src, dst);
    }
  }
  std::vector<uint16_t> rank(ds);
  std::vector<uint32_t> sorted_entries(ds);
  for (size_t k = 0; k < ds; ++k) {
    const uint32_t orig = static_cast<uint32_t>(keys_a[k] & 0xFFFF);
    rank[orig] = static_cast<uint16_t>(k);
    sorted_entries[k] = entries[orig];
  }
  for (size_t i = 0; i < n; ++i) idx[i] = rank[idx[i]];

  std::vector<uint8_t> plain(6 + ds * 4 + n * 2);
  plain[0] = 1;
  plain[1] = 2;  // residuals are always 2 bytes (split into two planes)
  for (int b = 0; b < 4; ++b) plain[2 + b] = static_cast<uint8_t>(ds >> (8 * b));
  static_assert(std::endian::native == std::endian::little, "format is little-endian");
  std::memcpy(plain.data() + 6, sorted_entries.data(), ds * 4);

  // Pass 2: MED predict + zigzag + byte-split. Unlike decode, this reads the
  // finished idx plane, so there is no serial dependency: the branchless
  // clamp form of MED lets the compiler vectorize the row loop.
  uint8_t* lo = plain.data() + 6 + ds * 4;
  uint8_t* hi = lo + n;
  const auto emit = [&](size_t i, int32_t pred) {
    // Residual lives mod 2^16: wrap, then 16-bit zigzag (bijective for any
    // jump size — a 32-bit zigzag truncated to 16 bits is not).
    const int16_t s = static_cast<int16_t>(static_cast<uint16_t>(idx[i] - pred));
    const uint16_t z = static_cast<uint16_t>((s << 1) ^ (s >> 15));
    lo[i] = static_cast<uint8_t>(z);
    hi[i] = static_cast<uint8_t>(z >> 8);
  };
  if (n) emit(0, 0);
  for (uint32_t x = 1; x < w; ++x) emit(x, idx[x - 1]);
  for (uint32_t y = 1; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    emit(row, idx[row - w]);
    const uint16_t* up = idx.data() + row - w;
    const uint16_t* cur = idx.data() + row;
    for (uint32_t x = 1; x < w; ++x) {
      const int32_t a = cur[x - 1], b = up[x], c = up[x - 1];
      const int32_t mn = std::min(a, b), mx = std::max(a, b);
      const int32_t pred = std::clamp(a + b - c, mn, mx);  // == MED(a,b,c)
      const int16_t s = static_cast<int16_t>(static_cast<uint16_t>(cur[x] - pred));
      const uint16_t z = static_cast<uint16_t>((s << 1) ^ (s >> 15));
      lo[row + x] = static_cast<uint8_t>(z);
      hi[row + x] = static_cast<uint8_t>(z >> 8);
    }
  }
  return zstd_pack(plain.data(), plain.size(), level);
}

void dpred_decode(const uint8_t* comp, size_t comp_size, float* out, uint32_t w, uint32_t h, int /*level*/) {
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint8_t> plain = zstd_unpack(comp, comp_size);
  if (plain.empty()) fail("dpred_decode: empty payload");
  uint32_t* words = reinterpret_cast<uint32_t*>(out);
  if (plain[0] == 0) {
    if (plain.size() != 1 + n * 4) fail("dpred_decode: size mismatch");
    std::memcpy(words, plain.data() + 1, n * 4);
    return;
  }
  if (plain.size() < 6) fail("dpred_decode: truncated header");
  const size_t ds = get_u32(&plain[2]);
  if (plain.size() != 6 + ds * 4 + n * 2) fail("dpred_decode: size mismatch");
  static_assert(std::endian::native == std::endian::little, "format is little-endian");
  std::vector<uint32_t> dict(ds);
  std::memcpy(dict.data(), plain.data() + 6, ds * 4);
  const uint8_t* lo = plain.data() + 6 + ds * 4;
  const uint8_t* hi = lo + n;

  // Decode is inherently serial along a row (each prediction needs the pixel
  // just decoded), but the branchless clamp form of MED keeps the pipeline
  // free of mispredicted branches.
  std::vector<uint16_t> idx(n);
  const auto unstep = [&](size_t i, int32_t pred) -> uint16_t {
    const uint16_t z = static_cast<uint16_t>(lo[i] | (hi[i] << 8));
    const uint16_t r = static_cast<uint16_t>((z >> 1) ^ (~(z & 1) + 1));
    const uint16_t k = static_cast<uint16_t>(pred + r);
    idx[i] = k;
    if (k >= ds) fail("dpred_decode: bad index");
    words[i] = dict[k];
    return k;
  };
  if (n) unstep(0, 0);
  for (uint32_t x = 1; x < w; ++x) unstep(x, idx[x - 1]);
  for (uint32_t y = 1; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    int32_t left = unstep(row, idx[row - w]);
    const uint16_t* up = idx.data() + row - w;
    for (uint32_t x = 1; x < w; ++x) {
      const int32_t a = left, b = up[x], c = up[x - 1];
      const int32_t mn = std::min(a, b), mx = std::max(a, b);
      const int32_t pred = std::clamp(a + b - c, mn, mx);  // == MED(a,b,c)
      left = unstep(row + x, pred);
    }
  }
}

const std::vector<Method> kMethods = {
    {"store", store_encode, store_decode},
    {"zstd", zstd_encode, zstd_decode},
    {"bss", bss_encode, bss_decode},
    {"alp", alp_encode, alp_decode},
    {"alprd", alprd_encode, alprd_decode},
    {"dict", dict_encode, dict_decode},
    {"fpred", fpred_encode, fpred_decode},
    {"dpred", dpred_encode, dpred_decode},
};

}  // namespace

const std::vector<Method>& methods() { return kMethods; }

const Method* find_method(const std::string& name) {
  for (const auto& m : kMethods) {
    if (m.name == name) return &m;
  }
  return nullptr;
}

// ---- Self-describing public API ------------------------------------------
std::vector<uint8_t> encode_depth(const float* data, uint32_t width, uint32_t height,
                                  const std::string& method, int level) {
  const Method* m = find_method(method);
  if (!m) fail("encode_depth: unknown method");
  std::vector<uint8_t> payload = m->encode(data, width, height, level);

  std::vector<uint8_t> blob;
  blob.reserve(payload.size() + 32);
  const char magic[4] = {'D', 'P', 'C', '1'};
  blob.insert(blob.end(), magic, magic + 4);
  blob.push_back(static_cast<uint8_t>(method.size()));
  blob.insert(blob.end(), method.begin(), method.end());
  auto put32 = [&blob](uint32_t v) {
    for (int i = 0; i < 4; ++i) blob.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  };
  put32(static_cast<uint32_t>(level));
  put32(width);
  put32(height);
  blob.insert(blob.end(), payload.begin(), payload.end());
  return blob;
}

std::vector<float> decode_depth(const uint8_t* blob, size_t size, uint32_t* out_width, uint32_t* out_height) {
  if (size < 5 || std::memcmp(blob, "DPC1", 4) != 0) fail("decode_depth: bad magic");
  size_t pos = 4;
  const uint8_t name_len = blob[pos++];
  if (pos + name_len + 12 > size) fail("decode_depth: truncated header");
  std::string method(reinterpret_cast<const char*>(blob + pos), name_len);
  pos += name_len;
  auto get32 = [&blob, &pos]() {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(blob[pos++]) << (8 * i);
    return v;
  };
  const int level = static_cast<int>(get32());
  const uint32_t width = get32();
  const uint32_t height = get32();

  const Method* m = find_method(method);
  if (!m) fail("decode_depth: unknown method");

  std::vector<float> out(static_cast<size_t>(width) * height);
  m->decode(blob + pos, size - pos, out.data(), width, height, level);
  if (out_width) *out_width = width;
  if (out_height) *out_height = height;
  return out;
}

}  // namespace depthcodec

# depth_image_compression

[![ci](https://github.com/facontidavide/depth_image_compression/actions/workflows/ci.yaml/badge.svg)](https://github.com/facontidavide/depth_image_compression/actions/workflows/ci.yaml)

Lossless compression of raw **32FC1** (single-channel `float32`) depth images.
Bit-exact round trip: NaN payloads, ±Inf and −0.0 are preserved unchanged.

On a 260-frame 1920×1200 corpus recorded from a ZED camera, the default
method (`dpred`, ZSTD level 1, single thread on an i7-13700H) achieves:

| method | ratio | encode MB/s | decode MB/s |
|---|---|---|---|
| zstd:3 on raw floats (baseline) | 2.31 | 180 | 1011 |
| **dpred:1 (default)** | **5.82** | **614** | **914** |
| dpred:3 | 5.91 | 436 | 890 |

For scale: the ROS `compressedDepth` (16-bit PNG) transport encoding of the
same frames compresses 4.42× — `dpred` is ~25% smaller *and* lossless on the
floats.

## API

```cpp
#include "depth_codec/depth_codec.hpp"

std::vector<uint8_t> blob = depthcodec::encode_depth(img, width, height);  // dpred:1
uint32_t w, h;
std::vector<float> back = depthcodec::decode_depth(blob.data(), blob.size(), &w, &h);
```

The blob is self-describing (magic, method name, level, dimensions), so the
decoder needs nothing but the bytes. Other methods from the registry
(`store`, `zstd`, `bss`, `alp`, `alprd`, `dict`, `fpred`) can be selected via
the optional `method`/`level` arguments; they exist mainly for benchmarking.

## Layout

Two colcon packages:

```
depth_codec/                     the codec library (plain CMake, no ROS required)
├── include/depth_codec/         public header
├── src/                         implementation
├── test/                        round-trip unit tests (ctest)
├── benchmark/                   corpus benchmark + google-benchmark harness
└── conanfile.py                 Conan 2 recipe
dpred_image_transport/           ROS 2 image_transport plugin
```

## ROS 2: transparent depth compression (`dpred_image_transport`)

`dpred_image_transport` is an [image_transport](https://index.ros.org/p/image_transport/)
plugin: once installed, every `image_transport` publisher of a 32FC1 depth
topic transparently offers `<base_topic>/dpred` (a `sensor_msgs/CompressedImage`),
and subscribers (rviz2 included) can select the `dpred` transport. The blob is
self-describing, so bags recorded with it remain decodable standalone.

```bash
# republish an existing raw depth topic as dpred (e.g. for recording):
ros2 run image_transport republish --ros-args \
  -p in_transport:=raw  -p out_transport:=dpred \
  -r in:=/camera/depth/image -r out:=/relay/depth

# and back:
ros2 run image_transport republish --ros-args \
  -p in_transport:=dpred -p out_transport:=raw \
  -r in:=/relay/depth -r out:=/camera/depth/decoded
```

Non-32FC1 encodings are declined with an error log (16UC1 support is on the
roadmap — the codec pipeline handles it naturally). Verified end-to-end
(bit-exact through a raw → dpred → raw republish chain) on ROS 2 Jazzy and
Lyrical.

## Building

Only dependency: **libzstd** (and a C++20 compiler). ZSTD level is
intentionally capped at 3 by usage convention; `-march=native` is ON by
default for the AVX2 fast paths (`-DDEPTH_CODEC_MARCH_NATIVE=OFF` for
portable builds — a scalar fallback is used).

### Plain CMake (library only)

```bash
sudo apt install libzstd-dev
cmake -B build -S depth_codec -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build
```

### Conan (2.x, library only)

```bash
conan create depth_codec --build=missing    # builds, runs tests, packages
```

### ROS 2 colcon (Jazzy)

The repo is a plain-CMake colcon package (`package.xml`, build type `cmake`):

```bash
cd ~/ws/src && git clone https://github.com/facontidavide/depth_image_compression.git
cd ~/ws && rosdep install --from-paths src --ignore-src -y
colcon build
colcon test
```

CI runs both flows: see `.github/workflows/ci.yaml`.

## Benchmarks

`benchmark/bench` verifies bit-exactness and measures ratio/throughput for
any set of methods over a corpus of `.bin` files
(`uint32 width | uint32 height | float32 * w*h`, little-endian):

```bash
build/bench /path/to/corpus --methods zstd:3,dict:1,dpred:1,dpred:3
```

`benchmark/bench_dpred` is a google-benchmark harness over 10 frames spread
across the corpus (used to iterate on `dpred` optimizations):

```bash
build/bench_dpred /path/to/corpus --benchmark_filter=all10
```

---

# The `dpred` algorithm

## Why it works

Depth images have two properties the pipeline exploits in sequence:

1. **Low value cardinality.** A single frame typically contains only
   10–65k distinct 32-bit patterns (sensor quantization, invalid-pixel
   codes). Each pixel can therefore be replaced by a small dictionary index.
2. **Spatial smoothness.** Neighboring pixels have similar depth. Once the
   dictionary is sorted by *value*, index distance ≈ depth distance, so a
   2D predictor reduces most pixels to near-zero residuals.

Each stage removes the redundancy the previous one exposed:
dictionary (value redundancy) → value-sort (value proximity → numeric
proximity) → 2D prediction (spatial redundancy) → ZSTD (entropy coding of
what remains).

## Encoding pipeline

Everything operates on the floats as opaque 32-bit little-endian words, so
non-finite values need no special handling — NaN is just another pattern.

### 1. Build a per-image dictionary

One pass over the pixels assigns every distinct 32-bit pattern a first-seen
id and produces the index image `idx[n]` (`uint16`). If more than 65 536
distinct patterns are found, the encoder falls back to plain ZSTD of the raw
words (mode 0 below), so the method stays fully general.

### 2. Sort the dictionary by value, remap the ids

Dictionary entries are sorted by IEEE-754 **total order** using the monotone
bit map

```
ord(w) = (w >> 31) ? ~w : (w | 0x80000000)      // unsigned compare == float order
```

which handles negatives correctly and places NaN above +Inf. Every pixel's
id is remapped to its sorted rank. After this step the index image is a
quantized, monotone re-encoding of depth: close depths ⇒ close indices.
Invalid pixels (NaN) all share the single top index.

### 3. 2D prediction (MED / LOCO-I)

Each pixel's index is predicted from its already-visited neighbors
(raster order):

```
        c  b            a = left, b = up, c = up-left
        a  ?            pred = clamp(a + b − c, min(a,b), max(a,b))
```

The clamp form is exactly the JPEG-LS median-edge-detector: it follows
gradients on smooth surfaces and clamps at horizontal/vertical edges.
Borders: `pred = 0` at (0,0), `left` on the first row, `up` on the first
column.

The residual is wrapped and zigzag-coded **in 16-bit arithmetic**, which is
bijective for any jump size:

```
r = (idx − pred) mod 2^16          // int16 wrap
z = (r << 1) ^ (r >> 15)           // zigzag: 0,−1,1,−2,2… → 0,1,2,3,4…
```

(A 32-bit zigzag truncated to 16 bits is *not* bijective — residuals up to
±65535 need 17 bits — and corrupts frames with >32k distinct values.)

Smooth regions and the interiors of NaN holes produce residual 0; only
object boundaries and hole edges produce large values.

### 4. Byte-split + ZSTD

The `uint16` zigzag residuals are split into a low-byte plane and a
high-byte plane. The high plane is almost entirely zeros (residuals rarely
exceed ±127) and collapses to nothing; the low plane is dense in tiny
values. Header + dictionary + both planes are compressed with ZSTD
(level ≤ 3; level 1 is the speed/ratio sweet spot).

## Decoding

Un-ZSTD, read the dictionary, then scan pixels in raster order: compute the
same MED prediction from already-decoded neighbors, add the un-zigzagged
residual (mod 2^16) to recover the index, and look the word up in the
dictionary. Decode is inherently serial along a row (each prediction needs
the pixel just decoded), but branchless MED keeps the chain tight.

## Payload format

The public blob (`encode_depth`) wraps any method as:

```
'D' 'P' 'C' '1' | u8 name_len | name bytes | i32 level | u32 width | u32 height | payload
```

The `dpred` payload is a single ZSTD frame whose decompressed content is
(all little-endian, `n = width * height`):

| field        | size        | meaning                                    |
|--------------|-------------|--------------------------------------------|
| mode         | u8          | 0 = fallback (raw words follow), 1 = dict  |
| idx_bytes    | u8          | always 2                                   |
| dict_size    | u32         | number of dictionary entries `ds` ≤ 65536  |
| dict         | u32 × ds    | 32-bit patterns, sorted by float total order |
| lo plane     | u8 × n      | low bytes of zigzag residuals              |
| hi plane     | u8 × n      | high bytes of zigzag residuals             |

Mode 0 content is `u8 0` followed by the `4n` raw words.

## Implementation notes (encoder speed)

These affect speed only, never the format:

- **Bucketized SIMD-probed dictionary hash**: 8 keys per cache-line bucket,
  compared with one AVX2 `cmpeq` + `movemask`; a per-bucket count masks
  stale slots so clearing the table is free. This exists because a scalar
  probe loop was 3.5× slower — top-down profiling showed 40% of pipeline
  slots lost to branch mispredicts (the unpredictable "how many probes?"
  branch flushes the speculative loads behind it), not to cache misses.
- **Radix sort** (two 16-bit LSD passes) for the dictionary sort; a
  comparison sort was 41% of total encode time.
- **Branchless MED** (`clamp` form) — auto-vectorizes on encode, and removes
  mispredict stalls from the serial decode chain.
- Thread-local reused ZSTD contexts.

Encode time budget after these: ~57% ZSTD, ~32% dictionary pass, ~11% rest.

## Scope and limits

- The dictionary stage presumes low per-image cardinality. On full-precision
  float data (>65k distinct values) `dpred` falls back to plain ZSTD; the
  registry's `fpred` (MED prediction directly on `ord(w)`, no dictionary,
  ~2.5×) is the general-purpose alternative.
- Images are processed independently — no inter-frame state.
- Format is little-endian; the encoder asserts a little-endian host.

# depth_image_compression

[![ci](https://github.com/facontidavide/depth_image_compression/actions/workflows/ci.yaml/badge.svg)](https://github.com/facontidavide/depth_image_compression/actions/workflows/ci.yaml)

Lossless compression of raw **32FC1** (`float32`) depth images — bit-exact,
NaN/±Inf/−0.0 included — plus a ROS 2 `image_transport` plugin to use it
transparently on depth topics.

On a 260-frame 1920×1200 ZED corpus (single thread, i7-13700H):

| method | ratio | encode MB/s | decode MB/s |
|---|---|---|---|
| zstd:3 on raw floats | 2.31 | 180 | 1011 |
| **dpred:1 (default)** | **5.82** | **614** | **914** |

That is ~25% smaller than the (lossy) ROS `compressedDepth` PNGs of the same
frames. How it works: see **[ALGORITHM.md](ALGORITHM.md)**.

## Layout

```
depth_codec/              codec library (plain CMake, no ROS required)
dpred_image_transport/    ROS 2 image_transport plugin
pixi.toml                 pixi environments (ROS-free + RoboStack ROS 2)
```

## API

```cpp
#include "depth_codec/depth_codec.hpp"

std::vector<uint8_t> blob = depthcodec::encode_depth(img, width, height);
uint32_t w, h;
std::vector<float> back = depthcodec::decode_depth(blob.data(), blob.size(), &w, &h);
```

The blob is self-describing; the decoder needs nothing but the bytes.

## ROS 2 transport

With `dpred_image_transport` installed, every 32FC1 publisher transparently
offers `<base_topic>/dpred` (`sensor_msgs/CompressedImage`) and subscribers —
rviz2 included — can pick the `dpred` transport:

```bash
ros2 run image_transport republish --ros-args \
  -p in_transport:=raw -p out_transport:=dpred \
  -r in:=/camera/depth/image -r out:=/relay/depth
```

Non-32FC1 encodings are declined with an error log (16UC1 is on the roadmap).

## Building

```bash
pixi run test              # ROS-free: library + unit tests
pixi run -e ros test-ros   # library + plugin via colcon (RoboStack, no system ROS)
```

Or classic: `cmake -B build -S depth_codec` (needs `libzstd-dev`), or drop the
repo in a colcon workspace (`rosdep install` resolves dependencies). Note:
`-march=native` is ON by default; set `-DDEPTH_CODEC_MARCH_NATIVE=OFF` for
portable binaries.

## Benchmarks

```bash
.build/bench /path/to/corpus --methods zstd:3,dict:1,dpred:1   # ratio + losslessness
.build/bench_dpred /path/to/corpus --benchmark_filter=all10    # google-benchmark
```

Corpus format: `.bin` files of `u32 width | u32 height | float32 × w·h` (LE).

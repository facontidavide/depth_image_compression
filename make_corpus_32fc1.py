#!/usr/bin/env python3
"""
Reconstruct RAW 32FC1 depth images from the harvest bags and write them as
self-describing .bin files for the C++ codec benchmark.

The depth is stored in the bags as sensor_msgs/CompressedImage with
format "32FC1; compressedDepth": a 12-byte codec header
(int32 format, float32 depthQuantA, float32 depthQuantB) followed by a PNG of a
16-bit inverse-depth image. We invert that quantization back to float32:

    depth = depthQuantA / (q - depthQuantB)   for q != 0
    depth = NaN                               for q == 0  (invalid / no return)

This mirrors ROS compressed_depth_image_transport and yields realistic raw
float depth (NaN holes included). Downstream codecs must treat the result as
opaque float32 (no exploiting the 16-bit origin).

Output: corpus_32fc1/depth_NNNNNN.bin, numbered to match harvest_png/, each:
    [uint32 width][uint32 height][float32 * width*height]   (little-endian)
"""

import io
import os
import struct
import sys

import numpy as np
from PIL import Image
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
COMPRESSED_IMAGE = "sensor_msgs/msg/CompressedImage"

IN_DIR = "harvest_depth"
OUT_DIR = "corpus_32fc1"


def find_mcap(d):
    for f in sorted(os.listdir(d)):
        if f.lower().endswith(".mcap"):
            return os.path.join(d, f)
    return None


def reconstruct(decoded):
    """Return (width, height, float32 ndarray) or None if not a 32FC1 PNG depth."""
    fmt = getattr(decoded, "format", "")
    data = bytes(decoded.data)
    off = data.find(PNG_MAGIC)
    if not fmt.startswith("32FC1") or off < 0:
        return None
    # 12-byte compressedDepth header: int32 format, float32 A, float32 B
    _fmt_enum, quant_a, quant_b = struct.unpack_from("<iff", data, 0)
    img = Image.open(io.BytesIO(data[off:]))
    img.load()
    q = np.asarray(img)                      # uint16, shape (H, W)
    if q.dtype != np.uint16:
        q = q.astype(np.uint16)
    h, w = q.shape
    qf = q.astype(np.float32)
    valid = q != 0
    depth = np.full((h, w), np.nan, dtype=np.float32)
    # depth = A / (q - B); B is negative here so (q - B) > 0 for all valid q
    depth[valid] = np.float32(quant_a) / (qf[valid] - np.float32(quant_b))
    return w, h, depth, quant_a, quant_b


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    harvest_dirs = sorted(
        os.path.join(IN_DIR, d) for d in os.listdir(IN_DIR)
        if os.path.isdir(os.path.join(IN_DIR, d))
    )
    counter = 0
    total_raw = 0
    sample_printed = 0
    # deterministic order: dir, then topic (matches extract_depth_png numbering)
    for hd in harvest_dirs:
        mcap_path = find_mcap(hd)
        if mcap_path is None:
            print(f"WARN no mcap in {hd}", file=sys.stderr)
            continue
        frames = []  # (topic, w, h, depth)
        with open(mcap_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for schema, channel, message, decoded in reader.iter_decoded_messages():
                if schema is None or schema.name != COMPRESSED_IMAGE:
                    continue
                rec = reconstruct(decoded)
                if rec is None:
                    continue
                w, h, depth, qa, qb = rec
                frames.append((channel.topic, w, h, depth, qa, qb))
        for topic, w, h, depth, qa, qb in sorted(frames, key=lambda r: r[0]):
            counter += 1
            out_path = os.path.join(OUT_DIR, f"depth_{counter:04d}.bin")
            with open(out_path, "wb") as o:
                o.write(struct.pack("<II", w, h))
                o.write(depth.tobytes(order="C"))
            total_raw += w * h * 4
            if sample_printed < 3:
                finite = np.isfinite(depth)
                nan_frac = 1.0 - finite.mean()
                dmin = float(np.nanmin(depth)) if finite.any() else float("nan")
                dmax = float(np.nanmax(depth)) if finite.any() else float("nan")
                ndistinct = np.unique(depth[finite]).size if finite.any() else 0
                print(f"[sample] {os.path.basename(out_path)} {w}x{h} "
                      f"A={qa:.2f} B={qb:.2f} depth=[{dmin:.3f},{dmax:.3f}]m "
                      f"NaN={nan_frac*100:.1f}% distinct_valid={ndistinct}")
                sample_printed += 1

    print(f"\nWrote {counter} files to {OUT_DIR}/  "
          f"({total_raw/1e9:.2f} GB raw float payload)")


if __name__ == "__main__":
    main()

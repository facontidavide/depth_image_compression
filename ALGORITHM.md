# `dpred` — lossless compression of 32FC1 depth images

`dpred` is the default method behind `encode_depth()` / `decode_depth()`
(see `depth_codec.hpp`). It losslessly compresses single-channel `float32`
depth images, bit-exactly: NaN payloads, ±Inf and −0.0 all round-trip
unchanged. On the 260-frame 1920×1200 evaluation corpus it achieves
**5.82× compression at 614 MB/s encode / 914 MB/s decode** (single thread,
i7-13700H, ZSTD level 1), versus 2.31× at 180 MB/s for plain `zstd:3` on the
raw floats.

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

## Measured results (260 frames, 2.4 GB raw, single thread)

| method   | ratio | enc MB/s | dec MB/s | note                        |
|----------|-------|----------|----------|-----------------------------|
| zstd:3   | 2.31  | 180      | 1011     | baseline on raw floats      |
| dict:3   | 2.91  | 168      | 1157     | dictionary, no prediction   |
| dpred:1  | 5.82  | 614      | 914      | **default**                 |
| dpred:3  | 5.91  | 436      | 890      | slightly smaller, slower    |

For scale: the ROS `compressedDepth` PNGs these frames came from total
4.42× — `dpred` beats the original *lossy-transport* encoding by ~25%
while being bit-exact on the floats.

Reproduce with:

```
build/bench ../corpus_32fc1                      # ratio + losslessness, all methods
build/bench_dpred ../corpus_32fc1                # google benchmark, dpred:1 focus
```

## Scope and limits

- The dictionary stage presumes low per-image cardinality. On full-precision
  float data (>65k distinct values) `dpred` falls back to plain ZSTD; the
  registry's `fpred` (MED prediction directly on `ord(w)`, no dictionary,
  ~2.5×) is the general-purpose alternative.
- Images are processed independently — no inter-frame state.
- Format is little-endian; the encoder asserts a little-endian host.

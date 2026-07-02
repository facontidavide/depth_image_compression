#!/usr/bin/env python3
"""
Extract PNG-encoded 32FC1 depth images from a tree of harvest MCAP files.

Each harvest is a sub-directory of the input folder (default: harvest_depth/)
containing exactly one *.mcap file. The depth topics are published as
sensor_msgs/msg/CompressedImage with format "32FC1; compressedDepth". Their
`data` field is NOT a bare PNG: it begins with a 12-byte `compressedDepth`
codec header (an int enum + depthQuantA/depthQuantB floats) followed by the
real PNG bytes (a 16-bit single-channel image of the quantized inverse depth).

This script keeps only messages that
  (a) declare a "32FC1" format, AND
  (b) actually contain a PNG signature in their data (so RVL-encoded or other
      codecs are skipped, not silently mis-saved),
strips the leading codec header, and writes the embedded PNG verbatim into a
single flat output folder (default: harvest_png/) using a sequential numeric
suffix:  depth_000001.png, depth_000002.png, ...

A manifest.csv records the provenance of every numbered file (source harvest,
mcap file, topic, declared format, capture time, image geometry), plus a
_skipped.csv / printed summary of anything that did NOT qualify, so nothing is
lost silently.

Run with:
    uv run --with mcap --with mcap-ros2-support python3 extract_depth_png.py
Optional args: --input-dir, --output-dir, --basename, --jobs.
"""

import argparse
import csv
import os
import struct
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
COMPRESSED_IMAGE = "sensor_msgs/msg/CompressedImage"


def find_mcap(harvest_dir):
    """Return the single .mcap file inside a harvest directory, or None."""
    cands = sorted(
        f for f in os.listdir(harvest_dir) if f.lower().endswith(".mcap")
    )
    if not cands:
        return None
    return os.path.join(harvest_dir, cands[0])


def parse_ihdr(png_bytes):
    """Return (width, height, bit_depth, color_type_name) from a PNG's IHDR."""
    if png_bytes[8:16] != struct.pack(">I", 13) + b"IHDR":
        raise ValueError("not a PNG / missing IHDR")
    w, h, bit_depth, color_type = struct.unpack(">IIBB", png_bytes[16:26])
    names = {0: "grayscale", 2: "RGB", 3: "palette", 4: "gray+alpha", 6: "RGBA"}
    return w, h, bit_depth, names.get(color_type, str(color_type))


def scan_one(harvest_dir):
    """
    Open one harvest's mcap and classify every CompressedImage message.

    Returns a dict with:
      dir, mcap, error (or None),
      qualified: list of records for PNG-encoded 32FC1 depth topics,
      skipped:   list of records for CompressedImage that did not qualify.
    No image bytes are returned here (cheap metadata pass).
    """
    out = {"dir": harvest_dir, "mcap": None, "error": None,
           "qualified": [], "skipped": []}
    mcap_path = find_mcap(harvest_dir)
    if mcap_path is None:
        out["error"] = "no .mcap file found"
        return out
    out["mcap"] = mcap_path
    try:
        with open(mcap_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for schema, channel, message, decoded in reader.iter_decoded_messages():
                if schema is None or schema.name != COMPRESSED_IMAGE:
                    continue
                fmt = getattr(decoded, "format", "")
                data = bytes(decoded.data)
                offset = data.find(PNG_MAGIC)
                is_32fc1 = fmt.startswith("32FC1")
                if is_32fc1 and offset >= 0:
                    w = h = bd = None
                    ct = ""
                    try:
                        w, h, bd, ct = parse_ihdr(data[offset:])
                    except Exception as e:  # PNG header unexpectedly malformed
                        out["skipped"].append({
                            "topic": channel.topic, "format": fmt,
                            "reason": f"PNG header parse failed: {e}",
                            "log_time": message.log_time,
                        })
                        continue
                    out["qualified"].append({
                        "topic": channel.topic,
                        "format": fmt,
                        "log_time": message.log_time,
                        "png_offset": offset,
                        "png_bytes": len(data) - offset,
                        "width": w, "height": h,
                        "bit_depth": bd, "color_type": ct,
                    })
                else:
                    reason = []
                    if not is_32fc1:
                        reason.append(f"format={fmt!r} (not 32FC1)")
                    if offset < 0:
                        reason.append("no PNG signature (e.g. JPEG/RVL)")
                    out["skipped"].append({
                        "topic": channel.topic, "format": fmt,
                        "reason": "; ".join(reason),
                        "log_time": message.log_time,
                    })
    except Exception as e:
        out["error"] = f"{type(e).__name__}: {e}"
    return out


def extract_one(args):
    """
    Worker for the write pass. args = (mcap_path, [(topic, out_path), ...]).
    Re-opens the mcap once and writes each requested topic's embedded PNG.
    Returns list of (out_path, ok, detail).
    """
    mcap_path, wanted = args
    wanted_topics = {t for t, _ in wanted}
    out_by_topic = dict(wanted)
    results = []
    written = set()
    try:
        with open(mcap_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for schema, channel, message, decoded in reader.iter_decoded_messages():
                if channel.topic not in wanted_topics or channel.topic in written:
                    continue
                data = bytes(decoded.data)
                offset = data.find(PNG_MAGIC)
                if offset < 0:
                    results.append((out_by_topic[channel.topic], False,
                                    "PNG signature vanished on re-read"))
                    written.add(channel.topic)
                    continue
                out_path = out_by_topic[channel.topic]
                with open(out_path, "wb") as o:
                    o.write(data[offset:])
                results.append((out_path, True, ""))
                written.add(channel.topic)
    except Exception as e:
        for t in wanted_topics - written:
            results.append((out_by_topic[t], False, f"{type(e).__name__}: {e}"))
    # anything we never saw
    for t in wanted_topics - written:
        results.append((out_by_topic[t], False, "topic not found on re-read"))
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input-dir", default="harvest_depth",
                    help="folder containing harvest_* sub-directories")
    ap.add_argument("--output-dir", default="harvest_png",
                    help="flat output folder for the numbered PNGs")
    ap.add_argument("--basename", default="depth",
                    help="filename stem; files are <basename>_<NNNNNN>.png")
    ap.add_argument("--jobs", type=int, default=min(8, (os.cpu_count() or 2)),
                    help="parallel worker processes")
    args = ap.parse_args()

    in_dir = args.input_dir
    out_dir = args.output_dir
    if not os.path.isdir(in_dir):
        sys.exit(f"input dir not found: {in_dir}")
    os.makedirs(out_dir, exist_ok=True)

    harvest_dirs = sorted(
        os.path.join(in_dir, d) for d in os.listdir(in_dir)
        if os.path.isdir(os.path.join(in_dir, d))
    )
    print(f"Found {len(harvest_dirs)} harvest directories in {in_dir}/")

    # ---- Pass 1: scan (parallel, metadata only) -------------------------
    scans = []
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        futs = {ex.submit(scan_one, d): d for d in harvest_dirs}
        for fut in as_completed(futs):
            scans.append(fut.result())
    scans.sort(key=lambda s: s["dir"])  # deterministic numbering order

    # ---- Assign sequential numbers in deterministic order ---------------
    total_qual = sum(len(s["qualified"]) for s in scans)
    width = max(4, len(str(total_qual)))
    manifest_rows = []
    write_jobs = []  # (mcap_path, [(topic, out_path), ...])
    counter = 0
    for s in scans:
        if not s["qualified"]:
            continue
        per_dir = []
        for q in sorted(s["qualified"], key=lambda r: r["topic"]):
            counter += 1
            fname = f"{args.basename}_{counter:0{width}d}.png"
            out_path = os.path.join(out_dir, fname)
            per_dir.append((q["topic"], out_path))
            manifest_rows.append({
                "index": counter,
                "filename": fname,
                "source_dir": os.path.basename(s["dir"]),
                "mcap_file": os.path.basename(s["mcap"]),
                "topic": q["topic"],
                "format": q["format"],
                "capture_time_ns": q["log_time"],
                "width": q["width"],
                "height": q["height"],
                "bit_depth": q["bit_depth"],
                "color_type": q["color_type"],
                "png_bytes": q["png_bytes"],
            })
        write_jobs.append((s["mcap"], per_dir))

    # ---- Pass 2: extract & write (parallel) -----------------------------
    write_ok, write_fail = 0, []
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        futs = {ex.submit(extract_one, job): job for job in write_jobs}
        for fut in as_completed(futs):
            for out_path, ok, detail in fut.result():
                if ok:
                    write_ok += 1
                else:
                    write_fail.append((out_path, detail))

    # ---- Manifest + skipped report --------------------------------------
    manifest_path = os.path.join(out_dir, "manifest.csv")
    with open(manifest_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(manifest_rows[0].keys()) if manifest_rows
                           else ["index"])
        w.writeheader()
        w.writerows(manifest_rows)

    skipped_path = os.path.join(out_dir, "_skipped.csv")
    with open(skipped_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["source_dir", "topic", "format", "reason", "capture_time_ns"])
        for s in scans:
            for sk in s["skipped"]:
                w.writerow([os.path.basename(s["dir"]), sk["topic"],
                            sk["format"], sk["reason"], sk["log_time"]])

    errored = [(os.path.basename(s["dir"]), s["error"]) for s in scans if s["error"]]

    # ---- Summary --------------------------------------------------------
    print("\n=== Extraction summary ===")
    print(f"harvest dirs scanned : {len(scans)}")
    print(f"qualified depth PNGs : {total_qual}")
    print(f"PNGs written OK      : {write_ok}")
    print(f"write failures       : {len(write_fail)}")
    print(f"manifest             : {manifest_path}")
    print(f"skipped (non-qual.)  : {sum(len(s['skipped']) for s in scans)}  -> {skipped_path}")
    if errored:
        print(f"\n!! {len(errored)} harvest(s) failed to open:")
        for d, e in errored:
            print(f"   {d}: {e}")
    if write_fail:
        print(f"\n!! {len(write_fail)} write failure(s):")
        for p, d in write_fail:
            print(f"   {os.path.basename(p)}: {d}")

    # nonzero exit if anything went wrong, so CI / callers can detect it
    if errored or write_fail:
        sys.exit(2)


if __name__ == "__main__":
    main()

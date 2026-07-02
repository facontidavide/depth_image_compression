// Benchmark + round-trip verification harness for 32FC1 depth codecs.
//
// For every .bin in the corpus (self-describing: uint32 w, uint32 h, then
// w*h float32), run each configured (method, level), verify the decode is
// BIT-EXACT (memcmp over the raw float bytes, so NaN/inf/-0 are checked too),
// and accumulate ratio + throughput. Throughput is reported in MB/s against
// the RAW float size (raw_bytes / time), separately for encode and decode.
//
// Usage:
//   bench [corpus_dir] [--max N] [--reps R] [--methods zstd:1,zstd:3,...]
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "depth_codec.hpp"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {

struct Image {
  uint32_t w = 0, h = 0;
  std::vector<float> data;
  size_t raw_bytes() const { return static_cast<size_t>(w) * h * sizeof(float); }
};

bool load_image(const fs::path& p, Image& img) {
  std::ifstream f(p, std::ios::binary);
  if (!f) return false;
  uint32_t wh[2];
  f.read(reinterpret_cast<char*>(wh), sizeof(wh));
  if (!f) return false;
  img.w = wh[0];
  img.h = wh[1];
  const size_t n = static_cast<size_t>(img.w) * img.h;
  img.data.resize(n);
  f.read(reinterpret_cast<char*>(img.data.data()), n * sizeof(float));
  return static_cast<bool>(f);
}

struct Run {
  std::string method;
  int level;
  std::string label() const { return method + ":" + std::to_string(level); }
};

struct Stats {
  size_t raw = 0, comp = 0;
  double enc_s = 0, dec_s = 0;
  size_t files = 0, fails = 0;
};

std::vector<Run> parse_runs(const std::string& spec) {
  std::vector<Run> runs;
  size_t i = 0;
  while (i < spec.size()) {
    size_t comma = spec.find(',', i);
    std::string tok = spec.substr(i, comma == std::string::npos ? std::string::npos : comma - i);
    size_t colon = tok.find(':');
    if (colon == std::string::npos) {
      runs.push_back({tok, 0});
    } else {
      runs.push_back({tok.substr(0, colon), std::stoi(tok.substr(colon + 1))});
    }
    if (comma == std::string::npos) break;
    i = comma + 1;
  }
  return runs;
}

}  // namespace

int main(int argc, char** argv) {
  std::string corpus = "../corpus_32fc1";
  size_t max_files = 0;  // 0 = all
  int reps = 1;
  std::string methods_spec = "store,zstd:1,zstd:3,zstd:6,zstd:9,zstd:12,zstd:19";

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--max" && i + 1 < argc) max_files = std::stoul(argv[++i]);
    else if (a == "--reps" && i + 1 < argc) reps = std::stoi(argv[++i]);
    else if (a == "--methods" && i + 1 < argc) methods_spec = argv[++i];
    else if (a[0] != '-') corpus = a;
  }

  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(corpus)) {
    if (e.is_regular_file() && e.path().extension() == ".bin") files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());
  if (max_files && files.size() > max_files) files.resize(max_files);
  if (files.empty()) {
    std::fprintf(stderr, "no .bin files in %s\n", corpus.c_str());
    return 1;
  }

  std::vector<Run> runs = parse_runs(methods_spec);
  std::vector<Stats> stats(runs.size());

  std::printf("corpus: %s  files: %zu  reps: %d\n\n", corpus.c_str(), files.size(), reps);

  std::vector<float> decoded;
  for (const auto& p : files) {
    Image img;
    if (!load_image(p, img)) {
      std::fprintf(stderr, "failed to load %s\n", p.c_str());
      continue;
    }
    decoded.assign(img.data.size(), 0.0f);

    for (size_t r = 0; r < runs.size(); ++r) {
      const depthcodec::Method* m = depthcodec::find_method(runs[r].method);
      if (!m) {
        std::fprintf(stderr, "unknown method %s\n", runs[r].method.c_str());
        return 1;
      }
      std::vector<uint8_t> comp;
      double best_enc = 1e300, best_dec = 1e300;
      for (int rep = 0; rep < reps; ++rep) {
        auto t0 = Clock::now();
        comp = m->encode(img.data.data(), img.w, img.h, runs[r].level);
        auto t1 = Clock::now();
        m->decode(comp.data(), comp.size(), decoded.data(), img.w, img.h, runs[r].level);
        auto t2 = Clock::now();
        best_enc = std::min(best_enc, std::chrono::duration<double>(t1 - t0).count());
        best_dec = std::min(best_dec, std::chrono::duration<double>(t2 - t1).count());
      }
      const bool ok = std::memcmp(decoded.data(), img.data.data(), img.raw_bytes()) == 0;

      Stats& s = stats[r];
      s.raw += img.raw_bytes();
      s.comp += comp.size();
      s.enc_s += best_enc;
      s.dec_s += best_dec;
      s.files += 1;
      if (!ok) s.fails += 1;
    }
  }

  std::printf("%-12s %12s %8s %7s %12s %12s %10s\n",
              "method", "comp(MB)", "ratio", "%", "enc MB/s", "dec MB/s", "lossless");
  std::printf("%s\n", std::string(78, '-').c_str());
  for (size_t r = 0; r < runs.size(); ++r) {
    const Stats& s = stats[r];
    const double ratio = s.comp ? double(s.raw) / double(s.comp) : 0.0;
    const double pct = s.raw ? 100.0 * double(s.comp) / double(s.raw) : 0.0;
    const double enc_mbs = s.enc_s ? (double(s.raw) / 1e6) / s.enc_s : 0.0;
    const double dec_mbs = s.dec_s ? (double(s.raw) / 1e6) / s.dec_s : 0.0;
    std::printf("%-12s %12.1f %8.3f %7.2f %12.1f %12.1f %10s\n",
                runs[r].label().c_str(), s.comp / 1e6, ratio, pct, enc_mbs, dec_mbs,
                s.fails ? "FAIL" : "ok");
  }
  std::printf("\nraw payload total: %.1f MB across %zu files\n", stats[0].raw / 1e6, files.size());
  return 0;
}

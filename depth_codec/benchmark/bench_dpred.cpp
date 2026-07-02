// Google Benchmark micro-harness for dpred:1 — the optimization target.
//
// Loads 10 real frames spread evenly across the corpus and registers one
// encode and one decode benchmark per frame (plus an "all10" aggregate).
// Round-trip losslessness is asserted once per frame at startup, outside the
// timed regions, so a broken optimization fails loudly instead of ranking.
//
// Usage:
//   bench_dpred [corpus_dir] [--benchmark_filter=encode] [...]
// corpus_dir defaults to ../corpus_32fc1 (run from the build directory).
#include <benchmark/benchmark.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "depth_codec/depth_codec.hpp"

namespace fs = std::filesystem;

namespace {

constexpr int kLevel = 1;  // dpred:1
constexpr size_t kSamples = 10;

struct Sample {
  std::string name;
  uint32_t w = 0, h = 0;
  std::vector<float> data;
  std::vector<uint8_t> comp;  // pre-encoded once, input for decode benchmarks
  size_t raw_bytes() const { return static_cast<size_t>(w) * h * sizeof(float); }
};

std::vector<Sample> g_samples;
const depthcodec::Method* g_dpred = nullptr;

bool load_bin(const fs::path& p, Sample& s) {
  std::ifstream f(p, std::ios::binary);
  if (!f) return false;
  uint32_t wh[2];
  f.read(reinterpret_cast<char*>(wh), sizeof(wh));
  if (!f) return false;
  s.w = wh[0];
  s.h = wh[1];
  s.data.resize(static_cast<size_t>(s.w) * s.h);
  f.read(reinterpret_cast<char*>(s.data.data()), s.data.size() * sizeof(float));
  s.name = p.stem().string();
  return static_cast<bool>(f);
}

bool load_samples(const std::string& corpus) {
  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(corpus))
    if (e.is_regular_file() && e.path().extension() == ".bin") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  if (files.size() < kSamples) {
    std::fprintf(stderr, "need >= %zu .bin files in %s, found %zu\n", kSamples, corpus.c_str(),
                 files.size());
    return false;
  }
  // 10 frames spread evenly across the dataset (deterministic).
  for (size_t k = 0; k < kSamples; ++k) {
    const size_t i = k * (files.size() - 1) / (kSamples - 1);
    Sample s;
    if (!load_bin(files[i], s)) {
      std::fprintf(stderr, "failed to load %s\n", files[i].c_str());
      return false;
    }
    g_samples.push_back(std::move(s));
  }
  // Pre-encode and assert bit-exact round trip once, outside timing.
  for (Sample& s : g_samples) {
    s.comp = g_dpred->encode(s.data.data(), s.w, s.h, kLevel);
    std::vector<float> back(s.data.size());
    g_dpred->decode(s.comp.data(), s.comp.size(), back.data(), s.w, s.h, kLevel);
    if (std::memcmp(back.data(), s.data.data(), s.raw_bytes()) != 0) {
      std::fprintf(stderr, "LOSSLESS FAIL on %s\n", s.name.c_str());
      return false;
    }
  }
  return true;
}

void bm_encode(benchmark::State& state, const Sample& s) {
  size_t comp_size = 0;
  for (auto _ : state) {
    std::vector<uint8_t> out = g_dpred->encode(s.data.data(), s.w, s.h, kLevel);
    benchmark::DoNotOptimize(out.data());
    comp_size = out.size();
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(s.raw_bytes()));
  state.counters["ratio"] = double(s.raw_bytes()) / double(comp_size);
}

void bm_decode(benchmark::State& state, const Sample& s) {
  std::vector<float> out(s.data.size());
  for (auto _ : state) {
    g_dpred->decode(s.comp.data(), s.comp.size(), out.data(), s.w, s.h, kLevel);
    benchmark::DoNotOptimize(out.data());
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(s.raw_bytes()));
}

void bm_encode_all(benchmark::State& state) {
  size_t comp_total = 0, raw_total = 0;
  for (auto _ : state) {
    comp_total = 0;
    for (const Sample& s : g_samples) {
      std::vector<uint8_t> out = g_dpred->encode(s.data.data(), s.w, s.h, kLevel);
      benchmark::DoNotOptimize(out.data());
      comp_total += out.size();
    }
  }
  for (const Sample& s : g_samples) raw_total += s.raw_bytes();
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(raw_total));
  state.counters["ratio"] = double(raw_total) / double(comp_total);
}

void bm_decode_all(benchmark::State& state) {
  std::vector<std::vector<float>> out(g_samples.size());
  for (size_t i = 0; i < g_samples.size(); ++i) out[i].resize(g_samples[i].data.size());
  size_t raw_total = 0;
  for (auto _ : state) {
    for (size_t i = 0; i < g_samples.size(); ++i) {
      const Sample& s = g_samples[i];
      g_dpred->decode(s.comp.data(), s.comp.size(), out[i].data(), s.w, s.h, kLevel);
      benchmark::DoNotOptimize(out[i].data());
    }
  }
  for (const Sample& s : g_samples) raw_total += s.raw_bytes();
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(raw_total));
}

}  // namespace

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);  // strips --benchmark_* flags
  const std::string corpus = argc > 1 ? argv[1] : "../corpus_32fc1";

  g_dpred = depthcodec::find_method("dpred");
  if (!g_dpred) {
    std::fprintf(stderr, "dpred not registered\n");
    return 1;
  }
  if (!load_samples(corpus)) return 1;

  for (const Sample& s : g_samples) {
    benchmark::RegisterBenchmark(("dpred1/encode/" + s.name).c_str(),
                                 [&s](benchmark::State& st) { bm_encode(st, s); });
    benchmark::RegisterBenchmark(("dpred1/decode/" + s.name).c_str(),
                                 [&s](benchmark::State& st) { bm_decode(st, s); });
  }
  benchmark::RegisterBenchmark("dpred1/encode/all10", bm_encode_all);
  benchmark::RegisterBenchmark("dpred1/decode/all10", bm_decode_all);

  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}

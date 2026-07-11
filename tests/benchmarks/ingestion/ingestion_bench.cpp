/*
 * @file ingestion_bench.cpp
 * @date 2026-07-11
 * @brief Benchmark suite for the QuickViz viz-plane ingestion contract
 *        (docs/ingestion_contract.md).
 *
 * Measures the producer-side cost of getting a data sample into a rendered
 * chart/scene, through the library's PUBLIC ingestion boundary only:
 *
 *   - RingBuffer<T>::Write            (plot time-series path, drop-oldest)
 *   - DataStream<T>::Push             (scene latest-only path, DoubleBuffer)
 *
 * Groups (see the contract's requirement I7 and the conformance table):
 *   micro     — burst max-rate per-sample cost, batched, alloc-gated
 *   paced     — per-sample tails at 1 kHz / 100 kHz, headless
 *   render    — same producers with a real Viewer + RtLinePlotWidget /
 *               stream-drain panel rendering at ~60 fps (I4)
 *   stalled   — same producers with the render thread deliberately stalled
 *               ~1 s per frame — the producer-never-blocks proof (I3)
 *   observer  — 1 kHz producer loop period jitter, ingestion attached with
 *               rendering vs ingestion compiled to no-ops (I5, the M10-A4
 *               shape)
 *   soak      — RSS growth over a long high-rate run (I6)
 *
 * Threading: the render loop runs on the MAIN thread (the GL rule,
 * CLAUDE.md section 8); producers run on background threads, exactly like a
 * real robot application. Producers stop the viewer via
 * Viewer::SetWindowShouldClose(), which GLFW documents as callable from any
 * thread.
 *
 * Alloc gate: the producer-side measured sections run under AllocProbe
 * (thread-local, producer thread only — the render plane is ALLOWED to
 * allocate, the ingestion boundary is not). A single allocation in a gated
 * section fails the run.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "alloc_probe.hpp"  // ONE TU per binary: defines operator new/delete
#include "bench_harness.hpp"

// Library headers are not warning-clean under -Wall -Wextra (pre-existing:
// unused parameter/variable in viewer input headers); keep the bench TU
// zero-warning without modifying src/.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "core/buffer/buffer_registry.hpp"
#include "core/buffer/ring_buffer.hpp"
#include "core/data_stream.hpp"
#include "plot/rt_line_plot_widget.hpp"
#include "viewer/panel.hpp"
#include "viewer/viewer.hpp"
#pragma GCC diagnostic pop

namespace {

using quickviz::DataStream;
using quickviz::Panel;
using quickviz::RingBuffer;
using quickviz::RtLinePlotWidget;
using quickviz::Viewer;
using quickviz::bench::AllocProbe;
using quickviz::bench::BenchResult;
using quickviz::bench::ComputeStats;

using Clock = std::chrono::steady_clock;
using std::chrono::nanoseconds;

// ---------------------------------------------------------------------------
// Sample types.
// ---------------------------------------------------------------------------

// Scalar time-series point: the type RtLinePlotWidget consumes (8 bytes).
using ScalarPoint = RtLinePlotWidget::DataPoint;
static_assert(sizeof(ScalarPoint) == 8, "scalar sample is 8 bytes");

// Typical pose / small-struct sample (64 bytes, POD).
struct PoseSample {
  double t;
  double x, y, z;
  double qw, qx, qy, qz;
};
static_assert(sizeof(PoseSample) == 64, "pose sample is 64 bytes");
static_assert(std::is_trivially_copyable_v<PoseSample>,
              "pose sample must be POD");

// Ring capacity: power of two, sized so a 100 kHz producer cannot wrap a
// 60 fps consumer's per-frame drain (1667 samples/frame << 8192).
constexpr std::size_t kRingCapacity = 8192;
using ScalarRing = RingBuffer<ScalarPoint, kRingCapacity>;
using PoseRing = RingBuffer<PoseSample, kRingCapacity>;

// Optimization sink so measured writes cannot be elided.
volatile double g_sink = 0.0;

// ---------------------------------------------------------------------------
// Run configuration + result registry.
// ---------------------------------------------------------------------------

bool g_smoke = false;
std::string g_filter;
std::string g_out_path = "ingestion_bench_report.json";
std::vector<BenchResult> g_results;
std::vector<std::string> g_gate_failures;

bool Enabled(const std::string& name) {
  return g_filter.empty() || name.find(g_filter) != std::string::npos;
}

double Scaled(double full, double smoke) { return g_smoke ? smoke : full; }

void Record(BenchResult result) {
  if (result.alloc_gated && result.allocations != 0) {
    g_gate_failures.push_back(result.name + ": " +
                              std::to_string(result.allocations) +
                              " allocation(s) in the measured section");
  }
  std::printf(
      "%-42s %-9s p50=%9.1f  p99=%9.1f  p99.9=%9.1f  max=%10.1f %s  "
      "(n=%zu, batch=%d, alloc=%llu%s%s)\n",
      result.name.c_str(), result.mode.c_str(), result.stats.p50,
      result.stats.p99, result.stats.p999, result.stats.max,
      result.unit.c_str(), result.stats.samples, result.batch,
      static_cast<unsigned long long>(result.allocations),
      result.alloc_gated ? " gated" : "",
      result.achieved_fps >= 0.0
          ? (", fps=" + std::to_string(result.achieved_fps)).c_str()
          : "");
  g_results.push_back(std::move(result));
}

// ---------------------------------------------------------------------------
// Measurement primitives.
// ---------------------------------------------------------------------------

inline double NsBetween(Clock::time_point a, Clock::time_point b) {
  return static_cast<double>(
      std::chrono::duration_cast<nanoseconds>(b - a).count());
}

struct MeasuredRun {
  std::vector<double> samples_ns;
  std::uint64_t allocations = 0;
};

// Burst micro: batched back-to-back ops per timed sample (amortizes the
// ~20 ns clock read; each recorded value is a batch MEAN).
template <typename Op>
MeasuredRun MeasureBurst(std::size_t n_samples, int batch, Op&& op) {
  MeasuredRun run;
  run.samples_ns.reserve(n_samples);
  AllocProbe probe;
  for (std::size_t s = 0; s < n_samples; ++s) {
    const auto t0 = Clock::now();
    for (int b = 0; b < batch; ++b) {
      op(s * static_cast<std::size_t>(batch) + static_cast<std::size_t>(b));
    }
    const auto t1 = Clock::now();
    run.samples_ns.push_back(NsBetween(t0, t1) / batch);
  }
  run.allocations = probe.allocations();
  return run;
}

// Paced producer: one op per period, per-op timing (true per-sample tails,
// including ~two clock reads of overhead). Pacing uses sleep_until for slow
// rates and a spin-wait for fast ones (sleep granularity >> 10 us periods).
// If the pacer falls far behind (e.g. this thread was preempted), the
// schedule is re-anchored instead of bursting to catch up.
template <typename Op>
MeasuredRun MeasurePaced(double rate_hz, std::size_t n_ops, Op&& op) {
  const auto period = nanoseconds(static_cast<std::int64_t>(1e9 / rate_hz));
  const bool spin = period < std::chrono::microseconds(200);
  MeasuredRun run;
  run.samples_ns.reserve(n_ops);
  AllocProbe probe;
  auto next = Clock::now() + period;
  for (std::size_t i = 0; i < n_ops; ++i) {
    if (spin) {
      while (Clock::now() < next) {
      }
    } else {
      std::this_thread::sleep_until(next);
    }
    const auto t0 = Clock::now();
    op(i);
    const auto t1 = Clock::now();
    run.samples_ns.push_back(NsBetween(t0, t1));
    next += period;
    if (t1 > next + 100 * period) {
      next = t1 + period;  // re-anchor after a long preemption
    }
  }
  run.allocations = probe.allocations();
  return run;
}

// ---------------------------------------------------------------------------
// Bench panels (public Panel API only — no library modifications).
// ---------------------------------------------------------------------------

// Paces the render loop by sleeping inside Draw(): ~60 fps for the render
// group (no vsync exists on an offscreen X server), or a deliberate multi-
// hundred-millisecond stall for the stalled group. Also measures the
// achieved frame rate.
class PacerPanel : public Panel {
 public:
  explicit PacerPanel(nanoseconds frame_period)
      : Panel("bench_pacer"), period_(frame_period) {}

  void Draw() override {
    const auto now = Clock::now();
    if (frames_.fetch_add(1, std::memory_order_relaxed) == 0) {
      first_frame_ns_.store(now.time_since_epoch().count(),
                            std::memory_order_relaxed);
      next_ = now + period_;
    }
    last_frame_ns_.store(now.time_since_epoch().count(),
                         std::memory_order_relaxed);
    std::this_thread::sleep_until(next_);
    next_ += period_;
    if (next_ < Clock::now()) {
      next_ = Clock::now() + period_;
    }
  }

  double AchievedFps() const {
    const auto frames = frames_.load(std::memory_order_relaxed);
    const double span_ns =
        static_cast<double>(last_frame_ns_.load(std::memory_order_relaxed) -
                            first_frame_ns_.load(std::memory_order_relaxed));
    if (frames < 2 || span_ns <= 0.0) {
      return -1.0;
    }
    return static_cast<double>(frames - 1) * 1e9 / span_ns;
  }

  std::uint64_t Frames() const {
    return frames_.load(std::memory_order_relaxed);
  }

 private:
  nanoseconds period_;
  Clock::time_point next_{};
  std::atomic<std::uint64_t> frames_{0};
  std::atomic<std::int64_t> first_frame_ns_{0};
  std::atomic<std::int64_t> last_frame_ns_{0};
};

// Render-plane consumer for the DataStream (scene latest-only) path: pulls
// at most one value per frame, exactly like the streaming_demo pre-draw
// callback.
class StreamDrainPanel : public Panel {
 public:
  explicit StreamDrainPanel(DataStream<PoseSample>* stream)
      : Panel("bench_stream_drain"), stream_(stream) {}

  void Draw() override {
    PoseSample latest;
    if (stream_->TryPull(latest)) {
      g_sink = latest.t;
    }
  }

 private:
  DataStream<PoseSample>* stream_;
};

// ---------------------------------------------------------------------------
// Windowed scenario runner: render loop on the main thread, producer on a
// background thread. Returns false when no display is available.
// ---------------------------------------------------------------------------

struct WindowedResult {
  MeasuredRun run;
  double achieved_fps = -1.0;
  bool display_available = true;
};

enum class DrainKind { kPlotWidget, kStreamPanel, kBoth };

template <typename ProducerFn>
WindowedResult RunWindowedScenario(const std::string& title,
                                   nanoseconds frame_period, DrainKind drain,
                                   ScalarRing* scalar_ring,
                                   DataStream<PoseSample>* pose_stream,
                                   ProducerFn producer) {
  WindowedResult out;
  std::unique_ptr<Viewer> viewer;
  try {
    viewer = std::make_unique<Viewer>(title, 640, 480);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[skip] cannot create window: %s\n", e.what());
    out.display_available = false;
    return out;
  }

  auto pacer = std::make_shared<PacerPanel>(frame_period);
  viewer->AddSceneObject(pacer);

  // Plot-widget drain: register the ring under a unique name, wire the
  // widget to it through the BufferRegistry (the documented plot path).
  const std::string buffer_name = "bench." + title;
  auto& registry = quickviz::BufferRegistry::GetInstance();
  std::shared_ptr<ScalarRing> ring_holder;
  if (drain == DrainKind::kPlotWidget || drain == DrainKind::kBoth) {
    // Registry stores shared_ptr; wrap the caller's ring without ownership.
    ring_holder = std::shared_ptr<ScalarRing>(scalar_ring, [](ScalarRing*) {});
    registry.AddBuffer<ScalarPoint>(buffer_name, ring_holder);
    auto plot = std::make_shared<RtLinePlotWidget>("bench_plot_" + title);
    plot->SetFixedHistory(10.0f);
    plot->SetYAxisRange(-1.5f, 1.5f);
    plot->AddLine("signal", buffer_name);
    viewer->AddSceneObject(plot);
  }
  if (drain == DrainKind::kStreamPanel || drain == DrainKind::kBoth) {
    viewer->AddSceneObject(std::make_shared<StreamDrainPanel>(pose_stream));
  }

  std::thread producer_thread([&]() {
    // Wait until the render loop is demonstrably live so the measurement
    // covers ingestion-with-rendering, not ingestion-before-rendering.
    while (pacer->Frames() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    out.run = producer();
    viewer->SetWindowShouldClose();
  });

  viewer->Show();
  producer_thread.join();
  out.achieved_fps = pacer->AchievedFps();
  if (!buffer_name.empty() && ring_holder) {
    registry.RemoveBuffer(buffer_name);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Producer ops.
// ---------------------------------------------------------------------------

inline ScalarPoint MakeScalar(std::size_t i) {
  const float t = static_cast<float>(i) * 1e-3f;
  return ScalarPoint{t, std::sin(t)};
}

inline PoseSample MakePose(std::size_t i) {
  const double t = static_cast<double>(i) * 1e-3;
  return PoseSample{t, t * 0.1, t * 0.2, 0.0, 1.0, 0.0, 0.0, 0.0};
}

// ---------------------------------------------------------------------------
// Group: micro (headless burst, batched, alloc-gated).
// ---------------------------------------------------------------------------

void BenchMicro() {
  const auto n_samples =
      static_cast<std::size_t>(Scaled(20000, 2000));
  constexpr int kBatch = 64;

  if (Enabled("micro/ring_write_scalar/burst")) {
    ScalarRing ring;  // overwrite-on-full (drop-oldest), the plot default
    auto run = MeasureBurst(n_samples, kBatch, [&](std::size_t i) {
      ring.Write(MakeScalar(i));
    });
    BenchResult r;
    r.name = "micro/ring_write_scalar/burst";
    r.group = "micro";
    r.mode = "headless";
    r.batch = kBatch;
    r.stats = ComputeStats(std::move(run.samples_ns));
    r.allocations = run.allocations;
    r.alloc_gated = true;
    r.notes = "max-rate burst; ring in steady-state overwrite";
    Record(std::move(r));
  }

  if (Enabled("micro/ring_write_pose/burst")) {
    PoseRing ring;
    auto run = MeasureBurst(n_samples, kBatch, [&](std::size_t i) {
      ring.Write(MakePose(i));
    });
    BenchResult r;
    r.name = "micro/ring_write_pose/burst";
    r.group = "micro";
    r.mode = "headless";
    r.batch = kBatch;
    r.stats = ComputeStats(std::move(run.samples_ns));
    r.allocations = run.allocations;
    r.alloc_gated = true;
    r.notes = "max-rate burst; ring in steady-state overwrite";
    Record(std::move(r));
  }

  if (Enabled("micro/stream_push_pose/burst")) {
    DataStream<PoseSample> stream;
    auto run = MeasureBurst(n_samples, kBatch, [&](std::size_t i) {
      stream.Push(MakePose(i));
    });
    BenchResult r;
    r.name = "micro/stream_push_pose/burst";
    r.group = "micro";
    r.mode = "headless";
    r.batch = kBatch;
    r.stats = ComputeStats(std::move(run.samples_ns));
    r.allocations = run.allocations;
    r.alloc_gated = true;
    r.notes = "latest-only overwrite, no consumer";
    Record(std::move(r));
  }
}

// ---------------------------------------------------------------------------
// Group: paced (headless per-op tails at declared rates, alloc-gated).
// ---------------------------------------------------------------------------

void BenchPaced() {
  struct Case {
    const char* name;
    double rate_hz;
    double seconds;
    bool pose;
  };
  const Case cases[] = {
      {"paced/ring_write_scalar/1kHz", 1e3, Scaled(8, 1), false},
      {"paced/ring_write_scalar/100kHz", 1e5, Scaled(2, 0.25), false},
      {"paced/stream_push_pose/1kHz", 1e3, Scaled(8, 1), true},
      {"paced/stream_push_pose/100kHz", 1e5, Scaled(2, 0.25), true},
  };

  for (const auto& c : cases) {
    if (!Enabled(c.name)) {
      continue;
    }
    const auto n_ops = static_cast<std::size_t>(c.rate_hz * c.seconds);
    MeasuredRun run;
    if (c.pose) {
      DataStream<PoseSample> stream;
      run = MeasurePaced(c.rate_hz, n_ops,
                         [&](std::size_t i) { stream.Push(MakePose(i)); });
    } else {
      ScalarRing ring;
      run = MeasurePaced(c.rate_hz, n_ops,
                         [&](std::size_t i) { ring.Write(MakeScalar(i)); });
    }
    BenchResult r;
    r.name = c.name;
    r.group = "paced";
    r.mode = "headless";
    r.rate_hz = c.rate_hz;
    r.stats = ComputeStats(std::move(run.samples_ns));
    r.allocations = run.allocations;
    r.alloc_gated = true;
    r.notes = "per-op timing incl. ~2 clock reads; no consumer";
    Record(std::move(r));
  }
}

// ---------------------------------------------------------------------------
// Groups: render (60 fps) and stalled (render thread sleeping ~1 s/frame).
// ---------------------------------------------------------------------------

void BenchWithRenderPlane(const std::string& group, nanoseconds frame_period,
                          const std::string& mode) {
  struct Case {
    std::string name;
    double rate_hz;
    double seconds;
    bool pose;
  };
  const Case cases[] = {
      {group + "/ring_write_scalar/1kHz", 1e3, Scaled(8, 2), false},
      {group + "/ring_write_scalar/100kHz", 1e5, Scaled(3, 1), false},
      {group + "/stream_push_pose/1kHz", 1e3, Scaled(8, 2), true},
  };

  for (const auto& c : cases) {
    if (!Enabled(c.name)) {
      continue;
    }
    const auto n_ops = static_cast<std::size_t>(c.rate_hz * c.seconds);

    ScalarRing ring;
    DataStream<PoseSample> stream;
    WindowedResult wr;
    if (c.pose) {
      wr = RunWindowedScenario(
          c.name, frame_period, DrainKind::kStreamPanel, &ring, &stream,
          [&]() {
            return MeasurePaced(c.rate_hz, n_ops, [&](std::size_t i) {
              stream.Push(MakePose(i));
            });
          });
    } else {
      wr = RunWindowedScenario(
          c.name, frame_period, DrainKind::kPlotWidget, &ring, &stream,
          [&]() {
            return MeasurePaced(c.rate_hz, n_ops, [&](std::size_t i) {
              ring.Write(MakeScalar(i));
            });
          });
    }
    if (!wr.display_available) {
      std::fprintf(stderr, "[skip] %s: no display available\n",
                   c.name.c_str());
      continue;
    }
    BenchResult r;
    r.name = c.name;
    r.group = group;
    r.mode = mode;
    r.rate_hz = c.rate_hz;
    r.stats = ComputeStats(std::move(wr.run.samples_ns));
    r.allocations = wr.run.allocations;
    r.alloc_gated = true;
    r.achieved_fps = wr.achieved_fps;
    r.notes = c.pose ? "render plane pulls latest once per frame"
                     : "RtLinePlotWidget drains ring every frame";
    Record(std::move(r));
  }
}

// ---------------------------------------------------------------------------
// Group: observer invisibility (I5). A 1 kHz producer loop with a fixed
// synthetic workload; measured quantity is the loop PERIOD JITTER
// |actual period - 1 ms| in ns. A/B: ingestion compiled to no-ops and no
// window, vs full-rate ingestion with a 60 fps render plane attached.
// ---------------------------------------------------------------------------

double SyntheticWorkload(std::size_t seed) {
  double x = static_cast<double>(seed % 1024) * 1e-6 + 1.0;
  for (int i = 0; i < 500; ++i) {
    x = x * 1.000000001 + 1e-9;
  }
  return x;
}

template <bool kIngest>
MeasuredRun ObserverLoop(std::size_t iterations, ScalarRing* ring,
                         DataStream<PoseSample>* stream) {
  constexpr auto kPeriod = std::chrono::milliseconds(1);
  MeasuredRun run;
  run.samples_ns.reserve(iterations);
  AllocProbe probe;
  auto next = Clock::now() + kPeriod;
  auto prev = Clock::now();
  for (std::size_t i = 0; i < iterations; ++i) {
    std::this_thread::sleep_until(next);
    next += kPeriod;
    const auto now = Clock::now();
    if (i > 0) {
      run.samples_ns.push_back(std::fabs(NsBetween(prev, now) - 1e6));
    }
    prev = now;
    g_sink = SyntheticWorkload(i);
    if constexpr (kIngest) {
      ring->Write(MakeScalar(i));
      stream->Push(MakePose(i));
    }
    if (Clock::now() > next + std::chrono::milliseconds(100)) {
      next = Clock::now() + kPeriod;  // re-anchor after a long preemption
    }
  }
  run.allocations = probe.allocations();
  return run;
}

void BenchObserver() {
  const auto iterations = static_cast<std::size_t>(Scaled(10000, 2000));

  if (Enabled("observer/noop_headless")) {
    auto run = ObserverLoop<false>(iterations, nullptr, nullptr);
    BenchResult r;
    r.name = "observer/noop_headless";
    r.group = "observer";
    r.mode = "headless";
    r.rate_hz = 1e3;
    r.stats = ComputeStats(std::move(run.samples_ns));
    r.allocations = run.allocations;
    r.alloc_gated = true;
    r.notes = "loop period jitter |period - 1ms|; ingestion compiled out";
    Record(std::move(r));
  }

  if (Enabled("observer/attached_render60")) {
    ScalarRing ring;
    DataStream<PoseSample> stream;
    auto wr = RunWindowedScenario(
        "observer_attached", nanoseconds(16666667), DrainKind::kBoth, &ring,
        &stream, [&]() {
          return ObserverLoop<true>(iterations, &ring, &stream);
        });
    if (!wr.display_available) {
      std::fprintf(stderr, "[skip] observer/attached_render60: no display\n");
      return;
    }
    BenchResult r;
    r.name = "observer/attached_render60";
    r.group = "observer";
    r.mode = "windowed";
    r.rate_hz = 1e3;
    r.stats = ComputeStats(std::move(wr.run.samples_ns));
    r.allocations = wr.run.allocations;
    r.alloc_gated = true;
    r.achieved_fps = wr.achieved_fps;
    r.notes =
        "loop period jitter |period - 1ms|; scalar+pose ingestion at 1 kHz, "
        "plot + stream drain rendering";
    Record(std::move(r));
  }
}

// ---------------------------------------------------------------------------
// Group: soak (I6). Scalar @ 100 kHz + pose @ 1 kHz into a rendering viewer
// for 60 s (smoke: 6 s); RSS sampled every 500 ms after a warm-up. Reported
// unit is KiB of growth relative to the post-warm-up baseline.
// ---------------------------------------------------------------------------

void BenchSoak() {
  if (!Enabled("soak/rss_growth")) {
    return;
  }
  const double seconds = Scaled(60, 6);
  const double warmup_s = seconds * 0.2;

  ScalarRing ring;
  DataStream<PoseSample> stream;

  std::unique_ptr<Viewer> viewer;
  try {
    viewer = std::make_unique<Viewer>("soak", 640, 480);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[skip] soak/rss_growth: no display (%s)\n",
                 e.what());
    return;
  }
  auto pacer = std::make_shared<PacerPanel>(nanoseconds(16666667));
  viewer->AddSceneObject(pacer);
  auto& registry = quickviz::BufferRegistry::GetInstance();
  auto ring_holder = std::shared_ptr<ScalarRing>(&ring, [](ScalarRing*) {});
  registry.AddBuffer<ScalarPoint>("bench.soak", ring_holder);
  auto plot = std::make_shared<RtLinePlotWidget>("bench_plot_soak");
  plot->SetFixedHistory(10.0f);
  plot->AddLine("signal", "bench.soak");
  viewer->AddSceneObject(plot);
  viewer->AddSceneObject(std::make_shared<StreamDrainPanel>(&stream));

  std::atomic<bool> stop{false};
  std::thread scalar_producer([&]() {
    const auto period = nanoseconds(10000);  // 100 kHz
    auto next = Clock::now() + period;
    std::size_t i = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      while (Clock::now() < next) {
      }
      next += period;
      ring.Write(MakeScalar(i++));
      if (Clock::now() > next + std::chrono::milliseconds(10)) {
        next = Clock::now() + period;
      }
    }
  });
  std::thread pose_producer([&]() {
    const auto period = std::chrono::milliseconds(1);  // 1 kHz
    auto next = Clock::now() + period;
    std::size_t i = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_until(next);
      next += period;
      stream.Push(MakePose(i++));
      if (Clock::now() > next + std::chrono::milliseconds(100)) {
        next = Clock::now() + period;
      }
    }
  });

  std::vector<double> growth_kib;
  growth_kib.reserve(static_cast<std::size_t>(seconds * 2) + 4);
  long rss_warm = -1;
  long rss_end = -1;
  std::thread sampler([&]() {
    const auto t0 = Clock::now();
    while (!stop.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      const double elapsed =
          std::chrono::duration<double>(Clock::now() - t0).count();
      const long rss = quickviz::bench::ReadVmRssKiB();
      if (elapsed < warmup_s || rss < 0) {
        continue;
      }
      if (rss_warm < 0) {
        rss_warm = rss;
      }
      rss_end = rss;
      growth_kib.push_back(static_cast<double>(rss - rss_warm));
      if (elapsed >= seconds) {
        stop.store(true, std::memory_order_relaxed);
        viewer->SetWindowShouldClose();
      }
    }
  });

  viewer->Show();
  stop.store(true, std::memory_order_relaxed);
  scalar_producer.join();
  pose_producer.join();
  sampler.join();
  registry.RemoveBuffer("bench.soak");

  BenchResult r;
  r.name = "soak/rss_growth";
  r.group = "soak";
  r.mode = "windowed";
  r.unit = "KiB";
  r.achieved_fps = pacer->AchievedFps();
  r.stats = ComputeStats(std::move(growth_kib));
  char note[160];
  std::snprintf(note, sizeof(note),
                "RSS growth vs post-warmup baseline; %.0f s soak, "
                "scalar@100kHz + pose@1kHz; rss_warm=%ld KiB, rss_end=%ld KiB",
                seconds, rss_warm, rss_end);
  r.notes = note;
  Record(std::move(r));
}

void PrintUsage(const char* argv0) {
  std::printf(
      "Usage: %s [--smoke] [--filter=<substring>] [--out=<report.json>]\n"
      "Windowed groups need a display; run under xvfb-run on headless "
      "machines (scripts/bench_ingestion.sh does this automatically).\n",
      argv0);
}

}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--smoke") {
      g_smoke = true;
    } else if (arg.rfind("--filter=", 0) == 0) {
      g_filter = arg.substr(9);
    } else if (arg.rfind("--out=", 0) == 0) {
      g_out_path = arg.substr(6);
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
      PrintUsage(argv[0]);
      return 2;
    }
  }

  auto hw = quickviz::bench::CaptureHardwareContext();
  std::printf("quickviz ingestion bench%s — cpu: %s, governor: %s, "
              "display: %s\n",
              g_smoke ? " (smoke)" : "", hw.cpu_model.c_str(),
              hw.governor.c_str(), hw.display.c_str());

  BenchMicro();
  BenchPaced();
  BenchWithRenderPlane("render60", nanoseconds(16666667), "windowed");
  BenchWithRenderPlane("stalled", nanoseconds(1000000000), "stalled");
  BenchObserver();
  BenchSoak();

  if (!quickviz::bench::WriteJsonReport(g_out_path, hw, g_results, g_smoke,
                                        g_gate_failures)) {
    std::fprintf(stderr, "failed to write report: %s\n", g_out_path.c_str());
    return 1;
  }
  std::printf("report: %s\n", g_out_path.c_str());

  if (!g_gate_failures.empty()) {
    std::fprintf(stderr, "ALLOC GATE FAILED:\n");
    for (const auto& f : g_gate_failures) {
      std::fprintf(stderr, "  %s\n", f.c_str());
    }
    return 1;
  }
  return 0;
}

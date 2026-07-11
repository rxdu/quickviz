# QuickViz Viz-Plane Ingestion Contract

- Status: **v0.1 — normative; budgets pinned from the first benchmark run (2026-07-11)**
- Benchmark suite: `tests/benchmarks/ingestion/` (run: `./scripts/bench_ingestion.sh`)
- Reference report: `tests/benchmarks/ingestion/results/2026-07-11_i7-8700_xvfb-llvmpipe.json`

## Why this contract exists

QuickViz is the third stage of the XMotion observability chain: telemetry instrumentation (~100 ns class, xmTelemetry S1 gate) → transport (xmMessaging publish 64 B p50 ~100–140 ns, M9 gate) → **visualization ingestion (this contract)**. The design intent for the whole chain is high-performance, low-overhead telemetry and data visualization: MHz-class ingestion with published nanosecond overhead, data ingestion fully decoupled from a fixed-rate render thread — *the robot must not be able to tell it is being watched*. The first two stages hold this bar with measured, regression-gated numbers; this document makes the same claims measurable for the viz plane.

This contract governs the **ingestion side only**. Rendering throughput (fps, draw calls, GPU upload strategies) is a separate concern with its own notes under `docs/notes/`.

## Definitions

- **Producer** — a robot/application thread (control loop, sensor callback, ROS2 subscriber, xmMessaging take-loop) that has a fresh data sample and wants it visualized. Producers are latency-critical; the viz plane is not allowed to perturb them.
- **Ingestion boundary** — the set of library calls a producer makes to hand a sample to the viz plane. Today these are exactly:
  - `RingBuffer<T, N>::Write(const T&)` — lossless-until-full time-series path consumed by plot widgets (`src/core/include/core/buffer/ring_buffer.hpp:182`);
  - `DataStream<T>::Push(...)` → `DoubleBuffer<T>::Write(...)` — latest-only path for scene/streaming data (`src/core/include/core/data_stream.hpp:56`, `src/core/include/core/buffer/double_buffer.hpp:26`);
  - `BufferRegistry::AddBuffer/GetBuffer` — **wiring-time only**; never called per sample (`src/core/include/core/buffer/buffer_registry.hpp:43`).
- **Render plane** — everything at frame cadence on the GL main thread: `Viewer::Show()` loop (`src/viewer/src/viewer.cpp:374`), panel `Draw()` including `RtLinePlotWidget::Draw()` draining ring buffers (`src/plot/src/rt_line_plot_widget.cpp:61`), `SceneManager` pre-draw callbacks pulling streams (`src/scene/src/scene_manager.cpp:121`), and renderable setters such as `PointCloud::SetPoints` (`src/scene/src/renderable/point_cloud.cpp:270`). Renderable setters are **render-plane calls, not ingestion calls**: they are unsynchronized and must only run on the render thread (via a pre-draw callback), as `sample/streaming_demo/main.cpp` demonstrates.

## Current ingestion architecture (facts, as of 2026-07-11)

**Path A — plot time series.** Producer writes `RtLinePlotWidget::DataPoint` (8 B) into a `RingBuffer<DataPoint, N>` registered by name in `BufferRegistry`. Per sample: one `std::lock_guard<std::mutex>` acquire/release plus an 8-byte copy into a member `std::array` — no allocation (`ring_buffer.hpp:182–194, 207`). Capacity `N` is a compile-time power of two (default 1024); usable capacity is `N-1`. On full: with `enable_overwrite_` (constructor default `true`, `ring_buffer.hpp:74`) the read index is advanced — **drop-oldest, silent, uncounted**; with overwrite disabled, `Write` returns 0 — **refusal, uncounted**. The render thread drains in `RtLinePlotWidget::Draw` (`rt_line_plot_widget.cpp:69–79`): `GetOccupiedSize()` then one `Read(pt)` per sample — one mutex acquisition per drained sample on the *same* mutex the producer takes — into a `ScrollingPlotBuffer` (fixed capacity 2048, overwriting, `scrolling_plot_buffer.cpp:35–42`) which ImPlot reads.

**Path B — latest-only stream.** Producer calls `DataStream<T>::Push`, which is `DoubleBuffer<T>::Write`: one mutex acquire, full `T` copy/move-assign into one of two slots, index swap, `ready` flag store, `notify_one()` (`double_buffer.hpp:26–36`). **Latest-only overwrite, silent, uncounted.** The render thread calls `TryPull` once per frame (`TryRead`, `double_buffer.hpp:52–62`): non-blocking, copies the whole `T` out *while holding the same mutex*. `DoubleBuffer::Read` (blocking condition-variable wait, `double_buffer.hpp:38`) exists but is not part of the render path and must never be.

**Path C — events (excluded).** `ThreadSafeQueue<T>` (`src/core/include/core/event/thread_safe_queue.hpp:49`) is an **unbounded** `std::queue` with per-push allocation, feeding `AsyncEventDispatcher`. It is an eventing utility, **not a data ingestion path**: using it for sample streams violates I2 and I6 below by construction.

Blocking analysis: neither `Write` ever waits on a condition — the only producer-side wait is mutex contention with the render-plane drain. For Path A the render thread holds the lock for one 8-byte copy at a time; for Path B it holds the lock for a full `sizeof(T)`-copy, so producer worst-case wait scales with the payload. A *stalled* render thread (stuck in a panel `Draw`, vsync-blocked, hidden window) holds no ingestion lock, so a full render stall cannot stall a producer — measured below.

## Requirements (normative)

Requirement IDs are `I1`–`I7`. Each is testable; the benchmark suite implements the tests. "Sample" below means a scalar time-series point (8 B) or a POD pose/small-struct (≤64 B) unless stated otherwise.

### I1 — Bounded per-sample ingestion cost

A producer's per-sample cost at the ingestion boundary is sub-microsecond at p99, at every supported rate up to at least 100 kHz per stream. Budgets (pinned 2026-07-11 from the first reference run, with headroom over measured values; re-pin requires a recorded decision):

| Budget | Operation | Metric | Bound | Measured (reference run) |
|---|---|---|---|---|
| B1 | scalar ring `Write`, burst (batch-64 mean) | p50 / p99.9 | ≤ 50 ns / ≤ 300 ns | 21.6 ns / 109 ns |
| B2 | pose (64 B POD) `DataStream::Push`, burst (batch-64 mean) | p50 / p99.9 | ≤ 50 ns / ≤ 300 ns | 10.3 ns / 14.3 ns |
| B3 | scalar ring `Write`, spin-paced 100 kHz, true per-op tails | p50 / p99 / p99.9 | ≤ 150 ns / ≤ 500 ns / ≤ 2 µs | 31 ns / 176 ns / 387 ns |
| B4 | pose `Push`, spin-paced 100 kHz, true per-op tails | p50 / p99 / p99.9 | ≤ 150 ns / ≤ 500 ns / ≤ 2 µs | 22 ns / 123 ns / 162 ns |

Methodology note: sleep-paced 1 kHz rows measure ~1.1 µs p50 on the reference machine in **every** mode including fully headless — that is post-wakeup DVFS/cache-warm cost on a `powersave` governor, a property of the producer's own sleep, not of the ingestion call. Rate-independence (I4) is therefore judged on like-for-like pacing, and budgets are pinned on burst and spin-paced rows.

### I2 — Allocation-free steady state

After wiring time (buffer registration, declared capacities) and a declared warm-up, ingestion of trivially-copyable samples performs **zero heap allocations** on the producer thread. Proven by allocation probe (the family S1 methodology) on every measured section; a single allocation fails the benchmark run. Non-POD payloads (e.g. `DataStream<PointCloudData>`) are only allocation-free if the application keeps slot capacities stable; the library does not yet give a way to pre-size `DoubleBuffer` slots — see conformance.

### I3 — Producer never blocks on the render plane

No ingestion call may wait for the render plane: no condition-variable waits, no frame-paced handshakes, no unbounded lock holds. A complete render stall (frame time ≥ 1 s) must leave producer-side ingestion tails statistically unchanged. Overflow policy must be **explicit and counted**: Path A is drop-oldest (or refusal when overwrite is disabled), Path B is latest-only overwrite — both MUST expose monotonic drop/overwrite/refusal counters queryable by the application. (Counters do not exist today — this is the contract's principal gap; see conformance.)

### I4 — Render-rate independence

Per-sample ingestion cost is invariant to render-plane state: rendering at 60 fps, rendering stalled, window hidden, or no window at all. Bound: windowed and stalled p99 within 3× of like-paced headless p99, and absolute p99 within B3/B4 bounds regardless of mode.

### I5 — Observer invisibility end-to-end

The M10-A4 acceptance shape, applied to the viz plane: a 1 kHz producer loop's period-jitter tails with full-rate ingestion *and* live rendering attached are statistically indistinguishable from the same loop with ingestion compiled to no-ops and no viz plane in the process. Judged at p99 on |period − 1 ms| with both distributions measured in the same session on the same core budget.

### I6 — Bounded memory

Every ingestion channel has a declared, wiring-time capacity: `RingBuffer` capacity is a compile-time constant; `DoubleBuffer`/`DataStream` hold exactly two slots; `ScrollingPlotBuffer` is a fixed overwriting window. Nothing on the data path may grow with time or with produced-sample count. Proven by soak: RSS growth ≈ 0 (≤ 2 MiB drift tolerance) over ≥ 60 s at 100 kHz scalar + 1 kHz pose with live rendering. `ThreadSafeQueue` is unbounded and therefore excluded from the data plane (Path C above).

### I7 — Measured claims

Every budget in this contract is benchmarked in-tree (`tests/benchmarks/ingestion/`), runnable with one command (`scripts/bench_ingestion.sh`), emitting a machine-readable JSON report that embeds the hardware context (CPU model, governor, kernel, RT patch, load, display/GL mode) — a context-less report is a failing run. Tails are reported as p50/p99/p99.9/max, never means alone. Once budgets are pinned (this document), a regression beyond them fails the run; wiring that gate into CI is an open item (see conformance).

## Accepted divergences from the transport-layer rules

- **The render thread is a legitimate hidden thread inside the viz plane.** xmMessaging's R3 forbids library-owned threads; QuickViz *is* the render loop. The divergence is contained: the render thread may allocate, block on vsync, and stall — the contract only forbids any of that leaking across the ingestion boundary to producers.
- **A mutex at the boundary is accepted at v0.1** (transport uses lock-free seqlock/SPSC structures). Accepted because the measured tails hold the budgets with margin; if a future payload or core-count profile breaches B1–B4, the remediation list (below) already names the lock-free replacement as the fix, not a budget relaxation.
- **`notify_one()` on the Path B write side is accepted** while no render-path code uses the blocking `Read` (no waiter → no futex wake). Introducing a waiter on the render path would move wake costs into the producer and is forbidden by I3.

## Conformance: quickviz vs this contract (2026-07-11 reference run)

Reference hardware: i7-8700 (12 threads), `powersave` governor, Linux 5.15.0-185-generic non-RT, offscreen X (Xvfb) with llvmpipe software GL, render loop paced to ~60 fps in-process (no vsync exists offscreen; pacing panel documented in the suite). Display mode does not affect ingestion-side numbers — demonstrated by the mode-invariance results themselves.

| Req | Status | Evidence / gap |
|---|---|---|
| I1 bounded cost | **conforms** | B1–B4 all hold with ≥2× margin (see budget table). |
| I2 alloc-free | **partial** | Zero allocations on every gated row (probe). Gap: no API to pre-size `DoubleBuffer`/`DataStream` slots for heap-owning payloads, so alloc-freedom for e.g. point clouds rests on application discipline, not a library guarantee. |
| I3 never blocks + counted overflow | **partial** | Never-blocks: proven — with the render thread stalled to ~1 fps, 100 kHz producer p99 = 84 ns (vs 126 ns at 60 fps, 176 ns headless). Gap (**principal**): drop-oldest, write-refusal, and latest-only overwrite are all silent and uncounted — no counter exists anywhere in `core/buffer`. Silent-loss violates the family's observability rule. |
| I4 render-rate independence | **conforms** | Spin-paced 100 kHz scalar p50/p99: headless 31/176 ns, 60 fps 76/126 ns, stalled 40/84 ns — invariant within noise, all within budget. |
| I5 observer invisibility | **partial (not proven on reference host)** | 1 kHz loop period jitter, no-op headless vs ingestion+60 fps rendering attached: p99 68 µs → 78 µs (+15%), but p50 1.0 µs → 7.6 µs and p99.9 115 µs → 991 µs. The degradation is CPU contention from **llvmpipe software rasterization** occupying cores (the paced I4 rows prove the ingestion boundary itself is mode-invariant at tens of ns), not boundary coupling — but the contract judges end-to-end invisibility, and on this host it does not hold. Must be re-proven on a hardware-GL host (and ideally a pinned-core / performance-governor setup) before the invisibility claim is made. |
| I6 bounded memory | **conforms (data path)** | Soak: RSS growth ≈ 0 KiB over the soak window at 100 kHz + rendering (see `soak/rss_growth` row). All data-path buffers bounded by construction. Caveat: `ThreadSafeQueue` (event path) is unbounded — kept out of the data plane by this contract, flagged for its own bound. |
| I7 measured claims | **partial** | Suite exists in-tree, one-command, JSON + hardware context, alloc-gated. Gap: not yet a CI regression gate (no pinned-reference comparison job like telemetry S1 / messaging M9). |

## Remediation list (ordered by severity against this contract)

1. **I3 — add overflow accounting** to `RingBuffer` (overwrite + refusal counters) and `DoubleBuffer`/`DataStream` (overwrite counter), monotonic, relaxed-atomic, queryable by the application and drainable into telemetry. This is the only *correctness*-class gap: today a robot losing plot samples cannot know it.
2. **I7 — CI regression gate**: pin `results/2026-07-11_*.json` as the reference, add a compare step (the xmMessaging `bench/compare.py` shape) to CI so budget breaches fail the pipeline.
3. **I2 — slot pre-sizing** for `DataStream<T>`/`DoubleBuffer<T>` (e.g. `Reserve(args...)` constructing both slots at declared capacity) so heap-owning payloads get a library-backed alloc-free steady state instead of a usage convention.
4. **I3/I1 hardening (optional, measured-first)**: replace the Path B mutex with a seqlock latest-slot for trivially-copyable `T` (the xmMessaging `LatestSlot` shape) to remove producer waits proportional to `sizeof(T)` during render-side copies. Only if a real payload profile breaches budgets — the contract forbids speculative rework.
5. **I5 — re-run the observer A/B on a hardware-GL host** (and ideally performance governor / pinned cores): the reference environment renders with llvmpipe, which burns CPU cores and perturbs the producer loop's scheduler tails (p99.9 115 µs → 991 µs) independently of the ingestion boundary. The invisibility number stays unpinned until measured where rendering does not compete for the producer's cores.

## How to run

```bash
./scripts/bench_ingestion.sh                 # full suite (~3 min), auto-xvfb when headless
./scripts/bench_ingestion.sh --smoke         # quick pass
./scripts/bench_ingestion.sh --filter=paced  # one group
```

The suite prints a human summary and writes `ingestion_bench_report.json` (schema `quickviz-ingestion-bench-v1`). Windowed groups skip with a recorded note when no display and no Xvfb are available.

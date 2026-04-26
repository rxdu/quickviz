# sample/streaming_demo — sensor data into a 3D scene, the right way

The canonical pattern for bringing data from a background thread (sensor
driver, ROS subscriber, network socket, processing pipeline) into a
QuickViz scene. Read `main.cpp` before you write your first sensor
callback — the same shape applies.

## What it does

Opens a window. A background thread generates a 2000-point colored
helix that rotates around the vertical axis at ~30 Hz and pushes each
new cloud into a `DataStream`. The render thread pulls the latest
cloud once per frame and updates a single `PointCloud` renderable. You
see a smoothly spinning spiral.

Close the window — the producer thread shuts down cleanly.

## The threading model

```
   ┌──────────────────┐                           ┌──────────────────┐
   │ producer thread  │                           │  render thread   │
   │ (sensor / ROS /  │   Push(PointCloudData)    │  (GLFW + GL)     │
   │  driver / your   │ ────────────────────────► │                  │
   │  callback)       │                           │   TryPull(out)   │
   └──────────────────┘                           │   SetPoints(out) │
                          DataStream<T>           └──────────────────┘
                          (latest-only,
                          lossy, never
                          blocks the
                          render loop)
```

Three rules captured by this layout:

1. **All OpenGL calls happen on the render thread.** The producer never
   touches GL or the renderable directly. It only writes to `stream`.
2. **The render thread never blocks waiting for data.** `TryPull`
   returns immediately with either the latest sample or "nothing new"
   — in the latter case the previous frame's cloud is reused.
3. **Intermediate samples are dropped silently.** If the producer
   pushes 30 clouds per second but the render loop runs at 144 Hz (or
   stutters under load), you only ever see the most recent cloud.
   Visualization rarely cares about yesterday's frame; this is what
   you want.

## When `DataStream<T>` is right

Use `DataStream<T>` when:

- You only care about the **latest** value (poses, sensor frames,
  status updates, processed outputs).
- The producer thread should **not block** on the consumer — losing
  intermediate values is acceptable.
- The data type is **trivially copyable / movable** as a whole unit
  (a struct, a `std::vector`, a small POD). Don't push references.

Use `RingBuffer<T>` (in `core/buffer/`) instead when:

- You need to **see every sample** (plotting time-series, replaying,
  recording, integration over time).
- The consumer should be able to fall behind without losing data
  (within the buffer's capacity).
- Order matters and you process samples one-by-one.

For most "show me the sensor on screen" robotics workflows, use
`DataStream<T>`.

## How to adapt this to a real sensor

Replace the synthetic producer (the `std::thread` block) with whatever
delivers your data:

```cpp
// instead of a synthetic loop, your driver/ROS callback pushes:
my_sensor.OnFrame([&stream](const RawFrame& frame) {
    auto cloud = ConvertToPointCloudData(frame);
    stream.Push(std::move(cloud));
});
```

Everything below the producer stays the same: one `DataStream<T>`,
one `SetPreDrawCallback` doing `TryPull` + `SetPoints`. The renderer
doesn't know or care where the data came from.

The forthcoming `bridges/ros2/` module will give you ready-made
producers for `sensor_msgs::PointCloud2`, `geometry_msgs::PoseStamped`,
and similar — so the boilerplate disappears for ROS2 users.

## Building & running

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
./build/bin/quickviz_streaming_demo
```

No external data files needed; all data is synthetic via
`quickviz::demo::SpiralCloud`.

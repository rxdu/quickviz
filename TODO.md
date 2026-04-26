# QuickViz Implementation Tracker

*Last Updated: April 26, 2026*
*Purpose: Track active work and concrete priorities. Terse and factual;
no marketing language. One bullet per outcome.*

## Mission

Visualization-first C++ library for robotics. The library renders,
displays, and interacts; apps built on top handle editor-style concerns
(undo/redo, project files, etc.). `sample/editor/` is the dogfood check
on library completeness.

The next chapter — turn this into a *robotics* visualization library
(not a general C++ one) by closing the gap to standard robotics
workflows: ROS2 streams, common primitives, live data, diagnostics.

---

## 🎯 Active priorities

Ordered by what unblocks the most downstream work.

### Next up — close the streaming-data loop
- [x] **`sample/streaming_demo/`** — rotating spiral pushed from a
      background thread at ~30 Hz, rendered via `DataStream<T>` and
      `SceneManager::SetPreDrawCallback`. Annotated README explains
      the threading model and DataStream-vs-RingBuffer choice. The
      reference users will copy when wiring sensor streams.

### Milestone — ROS2 integration (`bridges/ros2/`)
The single biggest user-facing gap. Without this QuickViz is a general
C++ vis library; with it, it's the robotics vis library.

**Hard rule for this and any ROS-dependent module**: ROS2 is an
**optional** CMake dep. The library must compile, link, and run
cleanly on systems where ROS2 is not installed — just without ROS
support. Use `find_package(... QUIET)` + early `return()` from the
module's CMakeLists when absent. No ROS headers / types may leak into
non-gated code paths. (See CLAUDE.md §4 "Optional external dependencies".)
- [ ] Stand up `bridges/` umbrella with `bridges/ros2/` as the first
      child (deferred from the earlier reorg — now justified by a real
      occupant). CMake-gated; absent ROS = silent skip.
- [ ] `sensor_msgs::PointCloud2` ↔ `PointCloud` renderable converter
- [ ] `geometry_msgs::PoseStamped` / `PoseArray` ↔ pose / trajectory
- [ ] `nav_msgs::OccupancyGrid` ↔ `OccupancyGrid` renderable (the
      renderable already exists as of April 2026)
- [ ] `tf2_msgs::TFMessage` ↔ `TfFrameTree` renderable (the renderable
      already exists as of April 2026)
- [ ] `visualization_msgs::Marker` / `MarkerArray` ↔ generic
      primitives passthrough
- [ ] Sample app demonstrating ROS2 streaming end-to-end. **Also
      CMake-gated** — must not be built when ROS2 is absent.

### Milestone — first standard robotics renderable
Pick one and ship it cleanly before the next. Don't backlog all three.
- [x] **`OccupancyGrid`** renderable — 2D map projected into the 3D
      scene. ROS-style int8 input (`-1` unknown, `0..100` occupancy
      percent). R8 texture + custom colormap shader. Visual test
      fixture in `scene/test/renderable/test_occupancy_grid.cpp`.
- [x] `TfFrameTree` renderable — named frames with parent/child
      transforms; renders RGB axes per frame plus optional gray
      connection lines. Walks parent chains at draw time; cycles +
      broken parent references handled defensively.
- [x] `Trajectory` extensions on the existing `Path` renderable —
      streaming-friendly `AddPoint(point, scalar)` overload,
      `EnableAutoColorRange()` for live trajectories, and a correctness
      fix for per-vertex scalar→color mapping under subdivided paths.
      (Pre-existing color modes `kVelocity` / `kTime` / `kCost` and
      `SetAnimationProgress` already covered the rest.)

### After ROS2 lands — diagnostics
- [ ] **HUD overlay**: frame time, draw call count, GPU memory, active
      tool, scene object count. Toggled by hotkey. ~1 day.
- [ ] **Structured logger** to replace the scattered `std::cerr` /
      `std::cout` calls in library code. Audit count (April 2026):
      ~232 occurrences in `src/`, concentrated in `scene` (179) and
      `viewer` (43). Lightweight implementation: ~100 LOC + a
      level/category enum, then a sed migration.
- [ ] Visible UI surface for shader/asset load failures (today these
      print to console and the user sees a black scene).

### After diagnostics — documentation site
- [ ] Doxygen API reference, generated and published via GitHub Pages
      from the same repo. ~1 day to set up.
- [ ] `docs/tutorial/01-quickstart.md` — walk through `sample/quickstart/`
- [ ] `docs/tutorial/02-editor.md` — walk through `sample/editor/`
- [ ] `docs/tutorial/03-streaming.md` — walk through `sample/streaming_demo/`
- [ ] `docs/tutorial/04-custom-renderable.md` — extension story
- [ ] `docs/tutorial/05-ros2.md` — once the bridge lands

---

## 📋 Backlog (deliberately deferred)

These are good ideas with concrete value but aren't on the critical
path right now. Promote one when there's a user pulling for it.

### Onramp / ergonomics
- [ ] **Layout presets** — `viewer::layout::SidebarLeft(width)`,
      `BottomDock(height)`, `Split(ratio)` returning configured `Box`
      trees. Removes the FlexGrow/FlexShrink magic-number incantation
      from samples. ~3 hours.
- [ ] **`viewer::AppState`** — saves window position, last camera
      viewpoint per scene, last-opened files, panel sizes beyond what
      `imgui.ini` covers. Loads on startup, saves on shutdown.
      ~1 day + a TOML/JSON dep.

### Tools / data lifecycle
- [ ] **Recording + replay** for `DataStream`s — `core/Record<T>` and
      `core/Replay<T>` capture / play back stream traffic to disk.
      Critical for deterministic testing and offline analysis. ~2-3
      days. Picks a binary format (raw, msgpack, capnproto).

### Performance
- [ ] **LOD system** for >1M point scenes (existing TODO, larger).
      Likely octree-based with per-tile streaming.

### Scaling / extension
- [ ] **Plugin / extension system** — runtime loading of renderables
      and tools via shared library + small C ABI registration. Heavy
      lift (~2-3 weeks); defer until a concrete user appears.

### Robotics gold demo
- [ ] **Compose** the items above into a single sample that looks
      impressive: robot driving through an occupancy grid with a
      streaming point cloud, trajectory, sensor frustum, and small
      dashboard. Doubles as a marketing screenshot and a
      feature-completeness check.

---

## 🔧 Library hooks (driven by `sample/editor`)

Additive only. Each one was logged when the editor sample wanted it
but worked around the absence. Re-evaluate against "is this a
visualization concern?" before merging.

- [ ] `SceneManager::GetObjectId(name)` / `GetObjectName(id)` — drop
      the stringly-typed name lookup the editor currently uses.
- [ ] `PointSelection::object_id` — selection callbacks no longer
      need string-equality on cloud_name.
- [ ] `PointCloud::SetActiveMask(span<bool>)` or
      `SetActiveIndices(...)` — editing a point cloud no longer
      requires rebuilding the full vertex buffer per command.
- [ ] Stable point identity so selections survive cloud mutations
      (today the editor must `ClearSelection()` after every rebuild
      because the tool tracks visible indices).

---

## 🐛 Known visualization gaps

- [ ] Selection support for `Arrow`, `Plane`, `Path`, `Triangle`,
      `Pose` primitives.
- [ ] `PCLLoaderTest.InvalidFileError` is failing — modern PCL no
      longer throws on corrupt PCDs; rewrite the test against current
      behavior.
- [ ] Move bundled fonts from `core/include/` to a top-level
      `resources/` directory (they're not a public-API concern).

---

## 🧹 Smaller cleanups

- [ ] Audit `GeometricPrimitive` base class (405-LOC header) — it
      bundles material system, render modes, and selection state for
      `Sphere`/`Cylinder`/`BoundingBox`. The header is overly large
      and possibly does too much. A real design review, not a quick
      rewrite — likely API-breaking.
- [ ] `src/scene/src/renderable/canvas.cpp` is 2069 LOC; split into
      cohesive sub-files (~500 LOC target per CLAUDE.md).
- [ ] Clean `sample/pointcloud_viewer/interactive_scene_manager.cpp`
      — confirmed leftovers from the deleted-editor migration:
      `HandleMouseInput()` is `return;` only (line 141-144), a
      "Legacy SelectionManager callback disabled" block (line 113-124),
      and a stale TODO at line 107. Either finish or remove each.
- [ ] Audit `sample/quickviz_demo_app/` after layout presets land.
      It uses `Viewer` directly (a multi-panel app — likely correct).
      The audit may conclude "no change"; the question is whether the
      layout setup could be tightened with the upcoming presets.

---

## ✅ Recently Completed

### April 2026

- ✅ **Path: per-pose orientation + `kPoseArrows`** — closes the gap
  between "path of positions" and "trajectory of poses". Optional
  `vector<glm::quat> orientations_` alongside `control_points_`,
  streaming-friendly `AddPoint(point, quat)` and
  `AddPoint(point, quat, scalar)` overloads, `SetOrientations` /
  `ClearOrientations` / `HasOrientations`. New `ArrowMode::kPoseArrows`
  draws arrows oriented by the per-pose quaternion (X-forward
  convention, ROS-style) rather than the path tangent — for use cases
  where heading differs from travel direction (parallel parking,
  drone yaw, manipulator end-effector). Maps cleanly onto the upcoming
  ROS2 `geometry_msgs::PoseArray` and `nav_msgs::Path` converters.
- ✅ **Path: trajectory streaming extensions** — `AddPoint(point, scalar)`
  overload pushes positions and scalar samples in lockstep for live
  trajectory feeds. `EnableAutoColorRange()` auto-fits the velocity /
  time / cost color range to the current `scalar_values_` so the
  mapping adapts as samples arrive. Fixed a long-standing bug where
  scalar-encoded colors on subdivided paths (smooth curve / Bezier /
  spline) misaligned because the per-vertex mapping indexed
  `scalar_values_` by raw vertex index; now uses fractional
  control-point parameter and lerps. Closes the third item under the
  "first standard robotics renderable" milestone.
- ✅ **Triangle moved to `scene/test/test_utils/`** — accidentally-public
  test scaffold removed from the public renderable API. New
  `scene_test_utils` library hosts test-only helpers; the 8 tests that
  used `Triangle` link against it instead of the main `scene` target.
  Headed off a related axis-vertex-generator refactor after closer
  inspection: the three axis-rendering classes (`CoordinateFrame`,
  `Pose`, `TfFrameTree`) draw genuinely different shapes (cone-arrowed
  vs. plain lines) in different coordinate frames; a shared helper
  would force a worse abstraction.
- ✅ **`TfFrameTree` renderable** — tree of named coordinate frames with
  parent/child transforms, ROS tf2-style. Each frame renders as RGB
  axes; gray lines connect parents to children when enabled. World
  transforms computed by walking parent chains; cycles and broken
  parent references handled defensively. Visual test fixture animates
  a 6-frame robot kinematics tree.
- ✅ **`OccupancyGrid` renderable** — 2D probabilistic grid as a
  textured quad in the XY plane. ROS-style API
  (`SetGrid(width, height, resolution, origin, vector<int8_t>)`,
  values: `-1` unknown / `0..100` occupancy). R8 texture with sentinel
  encoding (`byte 0 = unknown`, `byte 1..255 = free→occupied`); custom
  shader colormaps to configurable free/occupied/unknown colors.
  Manual visual test fixture in `scene/test/renderable/`.
- ✅ **`sample/streaming_demo/`** — canonical sensor-streaming pattern.
  Background producer rotates a 2000-point spiral around Z and pushes
  to `DataStream<PointCloudData>` at 30 Hz; render thread pulls in a
  `SetPreDrawCallback` and updates the renderable. ~50 LOC + a 60-line
  README on threading model and DataStream-vs-RingBuffer guidance.
- ✅ **CLAUDE.md rewrite** — tighter project contract, 470 → 308 lines.
  Final module map, library boundary rule, code style, threading model,
  decision heuristics. (commit `e637b8d`)
- ✅ **`quickviz::DataStream<T>`** — latest-only producer/consumer
  channel for streaming sensor data, header-only over `DoubleBuffer<T>`.
  7 unit tests including a threaded smoke test. (commit `22ef647`)
- ✅ **`sample/quickstart/`** — 18-line app demonstrating `SceneApp` +
  synthetic data; the "first 5 minutes" proof point. (commit `c6b9a2a`)
- ✅ **`quickviz::demo::*` synthetic data generators** — SpiralCloud,
  PlanarPointGrid, NoiseCloud, CubeMesh, Trajectory. 8 unit tests.
  (commit `135070a`)
- ✅ **`GlViewer` → `SceneApp` rename + reframe** as the 5-line
  quickstart facade. 17 renderable tests updated. (commit `557a27a`)
- ✅ **Module reorg by intent** — final layout `core, viewer, scene,
  plot, canvas, image, pcl_bridge`. One job per module, named after
  what users want to do (not which backend). Renames `imview→viewer`,
  `gldraw→scene`. Dissolved `widget` into `canvas` (Cairo) + `plot`
  (ImPlot widgets). Merged `cvdraw` into `image` along with cv_image
  widgets from `widget`. New `plot` module hosts ImPlot3D as well.
- ✅ **`sample/editor/` MVP** — vis+editing reference app on top of the
  library, built without any `src/` modifications. Acts as the dogfood
  check on library completeness.
- ✅ **Reshape: visualization-first re-anchor** — Removed the
  in-library state management module (`scenegraph`) and its sample
  (`object_management`). Locked the `src/ ↛ sample/` boundary in CI
  and CLAUDE.md.

### September 2025
- ✅ CameraController refactor (Strategy pattern, configurable
  parameters, utility methods)
- ✅ Input debug message cleanup
- ✅ GLDraw architecture review

### December 2024
- ✅ ThreadSafeQueue, BufferRegistry, AsyncEventDispatcher
  modernization

### September 2024
- ✅ Configurable camera controls (Modeling/FPS/CAD/Scientific styles)
- ✅ Unified input system with gamepad support
- ✅ Selection support for LineStrip, Mesh, Cylinder, BoundingBox

### Core Infrastructure
- ✅ CMake build system with module-private include layout
- ✅ GoogleTest integration
- ✅ Multi-layer point cloud system (60-100x batching speedup)
- ✅ GPU ID-buffer selection (16.5M point capacity)
- ✅ GeometricPrimitive template pattern

---

## 📊 Status Summary

**Branch**: `main` (post-PR-#28).
**Architecture**: Library = `core, viewer, scene, plot, canvas, image,
pcl_bridge`. Apps live in `sample/`. Editor / ROS-style frameworks live
above the library, never inside.
**Current focus**: Close the streaming loop with `sample/streaming_demo`,
then the ROS2 bridge milestone.

---

## 📝 Notes

- See `docs/notes/` for design deep-dives (rendering, picking, input).
- See `CLAUDE.md` for project guidelines and module boundaries.
- Update this file in the same change as the work itself; keep entries
  terse, factual, one bullet per outcome. No marketing language.
- An item moves from Active → Recently Completed only when the work is
  actually merged and tests pass — not "started" or "in flight."

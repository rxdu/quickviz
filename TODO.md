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
- [ ] **`sample/streaming_demo/`** — moving point cloud pushed from a
      background thread, rendered live via `DataStream<T>`. ~50 LOC.
      Validates the API shape and serves as the canonical streaming
      example before ROS2 work starts.

### Milestone — ROS2 integration (`bridges/ros2/`)
The single biggest user-facing gap. Without this QuickViz is a general
C++ vis library; with it, it's the robotics vis library.
- [ ] Stand up `bridges/` umbrella with `bridges/ros2/` as the first
      child (deferred from the earlier reorg — now justified by a real
      occupant).
- [ ] `sensor_msgs::PointCloud2` ↔ `PointCloud` renderable converter
- [ ] `geometry_msgs::PoseStamped` / `PoseArray` ↔ pose / trajectory
- [ ] `nav_msgs::OccupancyGrid` ↔ grid renderable
- [ ] `tf2_msgs::TFMessage` ↔ frame-tree visualization
- [ ] `visualization_msgs::Marker` / `MarkerArray` ↔ generic
      primitives passthrough
- [ ] CMake gating: ROS2 dep is optional; library still builds without
      it

### Milestone — first standard robotics renderable
Pick one and ship it cleanly before the next. Don't backlog all three.
- [ ] **`OccupancyGrid`** renderable — 2D map projected into the 3D
      scene. Strong companion to `nav_msgs::OccupancyGrid`. *Recommended
      first*: single-purpose, no existing partial implementation, lines
      up cleanly with ROS2 work.
- [ ] `TfFrameTree` renderable — animated transform tree with named
      frames and parent/child relationships. Likely composes multiple
      `CoordinateFrame` instances. Companion to tf2.
- [ ] `Trajectory` extensions on the existing `Path` renderable —
      timestamp coloring, velocity coloring, animated growth. `Path`
      already covers "render a 3D motion path"; this milestone adds the
      streaming/time-aware behaviors. (Don't create a new renderable —
      extend `Path`.)

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

# QuickViz TODO

*Last updated: 2026-04-26*

Tracker only. Mission, conventions, and module structure live in
[`CLAUDE.md`](CLAUDE.md). Older history: `git log`.

---

## 🎯 Active priorities (in order)

### 1. ROS2 bridge — `bridges/ros2/`
ROS2 is **optional** in CMake; the library must build and run without
it (CLAUDE.md §4 "Optional external dependencies").

- [ ] `bridges/` umbrella + `bridges/ros2/` first child, CMake-gated
- [ ] `sensor_msgs::PointCloud2` ↔ `PointCloud`
- [ ] `geometry_msgs::PoseStamped` / `PoseArray` ↔ `Path` (with orientations)
- [ ] `nav_msgs::OccupancyGrid` ↔ `OccupancyGrid`
- [ ] `tf2_msgs::TFMessage` ↔ `TfFrameTree`
- [ ] `visualization_msgs::Marker` / `MarkerArray` ↔ generic primitives
- [ ] Sample app demonstrating ROS2 streaming end-to-end (also gated)

### 2. Diagnostics
- [ ] HUD overlay: frame time, draw calls, GPU mem, scene object count, active tool
- [ ] Structured logger (replaces ~232 `std::cerr/cout` in `src/`, mostly in `scene` and `viewer`)
- [ ] Visible UI surface for shader/asset load failures

### 3. Documentation site
- [ ] Doxygen + GitHub Pages
- [ ] `docs/tutorial/01-quickstart.md`
- [ ] `docs/tutorial/02-editor.md`
- [ ] `docs/tutorial/03-streaming.md`
- [ ] `docs/tutorial/04-custom-renderable.md`
- [ ] `docs/tutorial/05-ros2.md` (after bridge lands)

---

## 📋 Backlog

### Onramp
- [ ] Layout presets — `viewer::layout::SidebarLeft(width)` etc.
- [ ] `viewer::AppState` — window pos, camera, panel sizes persisted

### Tools / data
- [ ] Recording + replay for `DataStream` — `core/Record<T>` / `core/Replay<T>`

### Performance
- [ ] LOD system for >1M point scenes (octree-based)

### Extension
- [ ] Plugin system (defer until concrete user)

### Robotics gold demo
- [ ] Composite sample: robot driving through occupancy grid with
      streaming cloud + trajectory + frustum + small dashboard

---

## 🔧 Library hooks (from sample/editor)

Re-evaluate each against "is this a visualization concern?" before merging.

- [ ] `SceneManager::GetObjectId(name)` / `GetObjectName(id)`
- [ ] `PointSelection::object_id` (drop string-equality on `cloud_name`)
- [ ] `PointCloud::SetActiveMask(span<bool>)` or `SetActiveIndices(...)`
- [ ] Stable point identity across cloud rebuilds (selections survive)

---

## 🐛 Visualization gaps

- [ ] Selection support for `Arrow`, `Plane`, `Path`, `Pose` primitives
- [ ] `PCLLoaderTest.InvalidFileError` — modern PCL no longer throws
      on corrupt PCDs; rewrite test against current behavior
- [ ] Move bundled fonts from `core/include/` to top-level `resources/`

---

## 🧹 Cleanups

- [ ] `GeometricPrimitive` 405-LOC header — design review (likely
      API-breaking)
- [ ] `scene/src/renderable/canvas.cpp` 2069 LOC — split (~500 target)
- [ ] `sample/pointcloud_viewer/interactive_scene_manager.cpp` —
      empty `HandleMouseInput()` line 141, disabled callback line 113,
      stale TODO line 107
- [ ] `sample/quickviz_demo_app/` — audit for boilerplate after layout
      presets land

---

## ✅ Recent (current iteration)

Use `git show <hash>` for full context.

- `a477e4f` Path: per-pose orientation + `ArrowMode::kPoseArrows`
- `a12d66d` Path: trajectory streaming extensions
            (`AddPoint(p, scalar)`, `EnableAutoColorRange`,
            scalar-color subdivision fix)
- `27a2583` Triangle moved to `scene/test/test_utils/`;
            new `scene_test_utils` library
- `1114780` ROS-optional CMake rule codified (CLAUDE.md §4)
- `54852f0` `TfFrameTree` renderable
- `f9c93a8` `OccupancyGrid` renderable
- `1b3022b` `sample/streaming_demo/`
- `f672d85` TODO audit
- `a8d126b` TODO roadmap rewrite
- `22ef647` `quickviz::DataStream<T>`
- `c6b9a2a` `sample/quickstart/`
- `135070a` `quickviz::demo::*` synthetic generators
- `e637b8d` CLAUDE.md rewrite (470 → 308 lines)
- `557a27a` `GlViewer` → `SceneApp` rename + reframe
- `64fad97` Module-reorg docs refresh
- `fcb47c5` New `image/` module (cvdraw + widget cv parts)
- `0479241` New `canvas/` module (Cairo bits of widget)
- `d44d5d3` New `plot/` module (ImPlot + ImPlot3D)
- `96d125f` `gldraw` → `scene` rename
- `06922c3` `imview` → `viewer` rename
- `2109140` Mission docs realigned, stale design docs deleted
- `079eb2b` `src/ ↛ sample/` boundary locked in CI + CLAUDE.md
- `af77ad4` Removed in-library state-mgmt module + bridge sample

# QuickViz Implementation Tracker

*Last Updated: April 26, 2026*
*Purpose: Track implementation status and priorities*

## Mission

QuickViz is a **visualization-first** C++ library for robotics. The library
provides building blocks (rendering, UI, selection, tools) that consuming
applications compose. Editor / app-level concerns (commands, undo/redo,
project files, history) live in `sample/` or downstream apps, never in `src/`.

`sample/editor/` (planned) is the dogfood check: if a vis+editing app cannot
be built on top of `src/` without modifying `src/`, the library is missing
a visualization-justified hook.

---

## 🎯 Active Work

### Reshape — bring the codebase back to visualization-first
- [x] Delete `src/scenegraph/` (state mgmt + command pattern + bridge that
      fabricated data — see commit `af77ad4`)
- [x] Delete `sample/object_management/` (demo of the deleted bridge)
- [x] CI `boundary-check` job: `src/` may not include from `sample/`
- [x] Update CLAUDE.md, archive stale design docs
- [x] Build `sample/editor/` MVP as the API completeness check
      (load PCD/PLY → render → select → DeleteSelectedPoints → undo/redo
      via app-side `CommandStack` → minimal history panel)
- [x] Module reorg: imview→viewer, gldraw→scene; widget split into
      canvas/plot/image; cvdraw merged into image; new plot module hosts
      ImPlot + ImPlot3D
- [ ] Add library hooks discovered while building the sample (additive only).
      Logged candidates from sample/editor MVP:
      - `SceneManager::GetObjectId(name) / GetObjectName(id)` so editors can
        avoid relying on stringly-typed cloud names.
      - `PointSelection::object_id` so selection callbacks don't need
        string-equality on cloud_name.
      - `PointCloud::SetActiveMask(span<bool>)` or `SetActiveIndices(...)` so
        editing a point cloud doesn't require rebuilding the full vertex
        buffer on every command (current MVP rewrites all visible points).
      - Stable point-identity for selection persistence across cloud
        mutations (today the editor must `ClearSelection()` on every
        rebuild because the tool tracks visible indices).
      Re-evaluate each one against the "is this a visualization concern?"
      bar before merging to src/.

### Known visualization gaps
- [ ] Selection support for Arrow, Plane, Path, Triangle, Pose primitives
- [ ] LOD system for >1M point scenes
- [ ] `PCLLoaderTest.InvalidFileError` is failing — modern PCL no longer
      throws on corrupt PCDs; rewrite the test to match current behavior
- [ ] Move bundled fonts from `core/include` to `resources/`
- [ ] Replace `std::cerr` / `std::cout` debug spew in library code with a
      lightweight logger (also audit for leftover noise after the deletions)

### Smaller cleanups
- [ ] `src/scene/src/renderable/canvas.cpp` is 2069 LOC; split into
      cohesive sub-files (~500 LOC target per CLAUDE.md)
- [ ] Audit `interactive_scene_manager.cpp` for disabled/legacy paths left
      over from the editor migration; either finish or remove

---

## ✅ Recently Completed

### April 2026
- ✅ **Module reorg by intent** — Final library layout:
  `core, viewer, scene, plot, canvas, image, pcl_bridge`. One job per
  module, named after what users want to do (not which backend it uses).
  Renamed `imview→viewer`, `gldraw→scene`. Dissolved `widget` into
  `canvas` (Cairo) + `plot` (ImPlot widgets). Merged `cvdraw` into
  `image` along with the cv_image widgets from `widget`. New `plot`
  module also hosts ImPlot3D.
- ✅ **`sample/editor/` MVP** — vis+editing reference app on top of
  the library, built without any `src/` modifications. Acts as the
  dogfood check on library completeness. Load PCD/PLY → select →
  DeletePoints with full undo/redo via a sample-private CommandStack.
- ✅ **Reshape: visualization-first re-anchor** — Removed the in-library
  state management module (`scenegraph`) and its sample (`object_management`).
  Locked the `src/ ↛ sample/` boundary in CI and CLAUDE.md. Editor concerns
  are now built on top of the library, not inside it. Stale architecture and
  design docs archived. (commits `af77ad4`, `079eb2b`)

### September 2025
- ✅ CameraController refactor (Strategy pattern, configurable parameters,
  utility methods)
- ✅ Input debug message cleanup
- ✅ GLDraw architecture review

### December 2024
- ✅ ThreadSafeQueue, BufferRegistry, AsyncEventDispatcher modernization

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

**Branch**: `feature-pointcloud_editing` (will rename once the editor sample
is in place)
**Focus**: Re-anchor the library on visualization, then build the editor
sample as the API check.
**Architecture**: Library = `core, viewer, scene, plot, canvas, image,
pcl_bridge` — one job per module, named by user intent. Apps live in
`sample/`.

---

## 📝 Notes

- See `docs/notes/` for design deep-dives (rendering, picking, input)
- See `CLAUDE.md` for project guidelines and module boundaries
- Update this file after finishing tasks; keep entries terse, factual,
  and one bullet per outcome

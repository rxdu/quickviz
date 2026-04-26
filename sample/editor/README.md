# sample/editor — vis+editing reference application

A minimal point-cloud editor built **on top of** the QuickViz library. This
sample exists primarily as the dogfood check on the library's API: if a
fully-featured vis+editing app cannot be built without modifying anything
under `src/`, the library is missing a visualization-justified hook.

## Scope (MVP)

- Load a `.pcd` / `.ply` via `pcl_bridge`
- Render in a viewport with the library's `SelectionManager` and
  `PointSelectionTool`
- One editing operation: delete the selected points
- Undoable via an editor-side `CommandStack` (lives in this sample,
  **not** in the library)
- Bound to `Ctrl+Z` / `Ctrl+Shift+Z` (or `Ctrl+Y`) and a "Delete Selected"
  button
- Minimal history panel showing the command stack

## Architecture in one paragraph

The editor holds the loaded data as the source of truth (an "alive" mask
over the original point array). The library's `PointCloud` renderable is
the **view**: every editing command mutates the alive mask and rebuilds the
visible cloud via `PointCloud::SetPoints`. Selection events from
`PointSelectionTool` arrive with visible-cloud indices, which the editor
maps back to original-array indices through a tiny lookup table. This keeps
commands simple and makes undo/redo trivially correct.

## Building & running

PCL is required. From the repo root:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
./build/bin/quickviz_editor data/pointcloud/cloud_uniform.pcd
```

## Boundary rule

This directory **may** include from the library's public headers
(`gldraw/`, `viewer/`, `pcl_bridge/`, `core/`). It must never be included
*by* anything in `src/`. The `boundary-check` CI job enforces this.

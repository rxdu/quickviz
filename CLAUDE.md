# QuickViz Project Guidelines

QuickViz is a C++ visualization library for robotics. Its goal is to let
users build a working UI/visualization tool with minimal boilerplate, then
keep growing the tool from there. This document is the contract for how the
codebase is organized and the rules contributors (human or agent) must
follow.

If anything in this file conflicts with code on disk, the code wins —
update this file in the same change.

---

## 1. Mission

- **Visualization first.** The library renders, displays, and interacts.
  It does not run editors, manage projects, persist documents, or model
  application state. Those belong in apps built *on top* of the library.
- **Building blocks, not a framework.** Provide composable pieces; let
  apps decide the architecture. No global state, no singletons exposed in
  public headers, no hidden lifecycles.
- **Generic over domain-specific.** Use graphics/robotics terms (`Mesh`,
  `PointCloud`, `Camera`) rather than app terms (`Map`, `Scan`,
  `Observer`). Keeps the library reusable across robotics applications.

---

## 2. Module Map

```
src/
├── core/          # events, buffers, threading helpers
├── viewer/        # window + panels + layout (GLFW + ImGui + Yoga)
├── scene/         # interactive 3D scene rendering (OpenGL)
├── plot/          # data charts (ImPlot 2D + ImPlot3D)
├── canvas/        # 2D vector drawing (Cairo)
├── image/         # image display & annotation (OpenCV, optional)
└── pcl_bridge/    # PCL adapter (optional)

third_party/       # imgui, implot, implot3d, stb, yoga, googletest
sample/            # reference applications built on top of the library
tests/             # cross-module integration tests
docs/              # design notes, architecture references
```

Pick a module by **what you want to do**, not by which backend it uses:

| I want to…                                    | Module        |
| --------------------------------------------- | ------------- |
| Open a window with panels                     | `viewer`      |
| Show a 3D scene (robots, point clouds, mesh)  | `scene`       |
| Plot a chart of data (2D or 3D)               | `plot`        |
| Draw a custom 2D figure                       | `canvas`      |
| Display or annotate camera images             | `image`       |
| Load PCL data                                 | `pcl_bridge`  |
| Use shared infrastructure                     | `core`        |

Domain modules stay tech-neutral so a future Vulkan-backed `scene` or
libpng-backed `image` slot in without churn. Inside each module,
implementation-specific types keep an explicit prefix (`Gl*` in `scene`,
`Cv*` in `image`).

### Quickstart classes worth knowing

- **`quickviz::Viewer`** (`viewer/viewer.hpp`) — the GLFW window + ImGui
  app shell. The fundamental primitive every QuickViz app starts from.
- **`quickviz::SceneApp`** (`scene/scene_app.hpp`) — five-line
  quickstart for a 3D viewer. Wraps `Viewer + GlScenePanel + grid +
  coordinate frame`. Use this when you want to display a scene fast;
  drop down to `Viewer` directly when you need richer layouts.

---

## 3. Library Boundary

`src/` is visualization-only. Editor / app-level concerns — undo/redo,
command stacks, project files, scene serialization, history panels, full
application frameworks — live in `sample/` or downstream apps and consume
the library, never the reverse.

- `sample/*` may include from `src/*/include/`.
- `src/*` must not include from `sample/`. Enforced by the
  `boundary-check` CI job.
- If a sample wants something from the library, add it as a small,
  visualization-justified hook to `src/` — never pull sample code in.

`sample/editor/` is the canonical example: a working point-cloud editor
built entirely on top of the library. It's also the dogfood check — if a
fully-featured vis+editing app cannot be built without modifying `src/`,
the library is missing a hook and we evaluate the gap deliberately.

---

## 4. Dependency Direction

Modules form a layered DAG. Lower layers never include from higher ones.

```
core ─► viewer ─► (scene | plot | canvas | image) ─► samples
                              │
pcl_bridge ◄──── scene ───────┘   (optional adapters depend outward)
```

- `core` depends on nothing else in the library.
- `viewer` depends on `core` (and ImGui/GLFW/Yoga from third_party).
- Visualization modules (`scene`, `plot`, `canvas`, `image`) depend on
  `core` + `viewer`. They do **not** depend on each other unless there
  is a concrete need (currently none do).
- Adapters (`pcl_bridge`, optional OpenCV in `image`) depend outward;
  no in-library code depends on them.

When tempted to add a cross-module dependency, ask: is this concept truly
shared, or am I leaking implementation? Prefer to widen the public API of
the lower module than to reach across at the same level.

### Optional external dependencies

Any module or component that depends on a heavyweight external SDK
(ROS2, PCL, OpenCV, vendor sensor SDKs) **must** be CMake-gated so the
library compiles cleanly without that SDK installed. The pattern:

- Use `find_package(<dep> QUIET)`. If absent, `return()` early from the
  module's `CMakeLists.txt`.
- Do not reference the optional dep's headers, types, or functions
  outside the gated module's source files.
- Document the dep in §5 Build System under "Optional".
- Library users without the dep get a working library with that
  feature missing, not a build failure.

This applies specifically to ROS2: any current or future module that
consumes ROS messages — `bridges/ros2/`, downstream converters,
sample apps that use them — must be optional. A user who has never
installed ROS must still be able to clone, configure, build, and run
the rest of the library.

---

## 5. Build System

Required: OpenGL 3.3+, GLFW3, GLM, Cairo.
Optional: OpenCV (enables `image`), PCL (enables `pcl_bridge`),
Google Benchmark, Valgrind, lcov.
Bundled in `third_party/`: ImGui, ImPlot, ImPlot3D, STB, Yoga, GoogleTest, GLAD.

```bash
git clone --recursive <repo>
git submodule update --init --recursive
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
make -j$(nproc)
ctest --output-on-failure
```

Common CMake options:

- `BUILD_TESTING` (default OFF) — build unit + integration tests
- `ENABLE_AUTO_LAYOUT` (default ON, requires C++20) — Yoga flexbox layout
- `VIEWER_WITH_GLAD` (default ON) — bundle GLAD as the GL loader
- `STATIC_CHECK` (default OFF) — run cppcheck during build

CI runs the `boundary-check` job before the full build matrix; failing it
blocks the rest of the pipeline.

---

## 6. Code Style

- **Standard**: C++17 (some C++20 features behind `ENABLE_AUTO_LAYOUT`).
- **Files**: `.cpp` for source, `.hpp` for headers. Snake_case filenames.
- **Style**: Google C++ Style; clang-format applied. Line length 100.
- **Naming**:
  - Types: `PascalCase`. Methods: `PascalCase()`. Members: `snake_case_`.
  - Implementation prefixes are deliberate signals: `Gl*` means "wraps an
    OpenGL handle"; `Cv*` means "uses OpenCV". Don't strip them when a
    type genuinely couples to that backend.
  - Module directories are snake_case; namespaces are `quickviz::*` (or
    a sub-namespace in samples).
- **No `using namespace` in headers.** Inside `.cpp` files, `using
  namespace quickviz` is acceptable.
- **Include order**: own header → standard library → third-party →
  project headers, separated by blank lines. Each block alphabetized.
- **Header guards**: `QUICKVIZ_<MODULE>_<NAME>_HPP` form is preferred; do
  not use `#pragma once` in public headers.
- **File size**: aim for ~500 LOC. Files significantly above that are
  candidates for splitting along internal seams.

---

## 7. Architecture Patterns

### Scene rendering
- `OpenGlObject` (`scene/interface/opengl_object.hpp`) is the base for
  anything rendered in 3D. Subclasses own GPU resources via RAII.
- `SceneManager` owns OpenGL objects keyed by name; `GlScenePanel` hosts
  the scene inside an ImGui window via render-to-texture.
- Per-pass: each draw path explicitly sets program, VAOs, depth, blend,
  and cull state — never inherit it from the previous pass.

### Selection
- Two-stage picking: GPU ID-buffer for object/point ID → CPU raycast for
  precise hit. Pick reads exactly one pixel; never large readbacks.
- `SelectionManager` and `PointSelectionTool` provide ready-made
  selection without an editor. Selection events are visualization
  events; if you want them to be undoable, layer a Command pattern on
  top in your app (`sample/editor/` shows how).

### Tools
- `InteractionTool` registers with `SceneManager`. One active tool at a
  time. Tools emit selection / hover / measurement events; they do not
  record history.

### Panels and layout
- `Panel` (in `viewer`) is the ImGui-based base for any UI panel.
- `Box` provides flexbox layout via Yoga (when `ENABLE_AUTO_LAYOUT`).
- For "give me a 3D viewer, fast", use `SceneApp`; for richer layouts,
  compose `Viewer` + panels + `Box` directly.

---

## 8. Threading Model

- **GL main-thread only.** All OpenGL calls and ImGui rendering happen
  on the render thread. Never make GL calls from background threads.
- **Background work** (file I/O, decoding, BVH builds, normal generation)
  produces immutable CPU buffers and queues a main-thread task to upload
  to GPU.
- **No frame stalls.** The render loop must never block on a background
  job. Use `core/buffer` (RingBuffer, DoubleBuffer) for handoff.
- `core/event/AsyncEventDispatcher` provides bounded async event delivery
  if the app needs it; do not roll a new threading framework into the
  library without explicit decision.

---

## 9. API Design Principles

- **Small surface, strong contracts.** Every public function should have
  a clear precondition, postcondition, and ownership story.
- **Explicit inputs.** Functions take `projection`, `view`, sizes, IDs;
  no hidden globals.
- **Clear ownership.** Prefer `std::unique_ptr` in APIs; use
  `std::shared_ptr` only for true sharing. Return raw pointers/refs only
  when ownership is documented as remaining elsewhere.
- **Narrow types.** Prefer `std::span<T const>` for read-only bulk data.
  Avoid leaking STL container choices in public headers.
- **Unit awareness.** Document meters, seconds, radians. Never assume
  degrees or "pixels" for world-scale values.
- **CPU double, GPU float.** Keep CPU transforms in `double`; upload as
  `float` to shaders. Caller-visible math is `double` unless documented
  otherwise.
- **No upstream types in public headers.** `pcl::*` and `cv::*` types are
  hidden behind the bridges/adapters.

---

## 10. Error Handling

- **Validate at boundaries.** Library entry points check their inputs
  and report failures precisely. Internal helpers may trust callers.
- **Fail loud, fail early.** Throw on contract violations; do not return
  silently corrupt data. `std::runtime_error` is the default; project
  may add specific types where that helps.
- **Graceful rendering degradation.** On shader/asset load failure, log
  the error and render a fallback (magenta material) so the rest of the
  scene stays visible.
- **No silent error suppression.** `catch(...) {}` requires a comment
  explaining why and what's lost. The deleted `SceneManagerBridge` is
  the cautionary tale: silent fallbacks that fabricated data hid real
  failures.

---

## 11. Decision Heuristics

When designing or reviewing changes, prefer:

- **Hide if it leaks.** If a type couples callers to OpenGL or third-party
  details, hide it behind an interface.
- **Runtime over compile-time.** Templates earn their keep with concrete
  performance or type-safety wins. Otherwise, prefer runtime polymorphism.
- **More passes over more state.** If a render path branches heavily on
  state, split it into separate passes.
- **CPU for precision/topology, GPU for per-pixel.** BVH/KD on CPU; ID
  buffers and shading on GPU.
- **Queued over immediate** for GPU-resource creation off the main thread.
- **Generic over specific.** Robotics/graphics vocabulary, not
  application vocabulary.
- **Library stays a library.** When in doubt, push the feature to
  `sample/` and let it prove it belongs in `src/`.

---

## 12. Tests

- Unit tests live next to the module: `src/<module>/test/`.
- Cross-module integration tests live in `tests/integration/`.
- GoogleTest is the framework; new tests follow the patterns of the
  existing ones.
- Renderable tests use `SceneApp` as their harness.
- Aim for 80% coverage overall. Designated hot paths should aim for 100%.

---

## 13. Commits & Documentation

- Commit messages: `type(scope): subject` form. Explain *why* in the
  body, not what the diff already shows. One logical change per commit.
- Don't amend pushed commits. Force-push to `main` is forbidden without
  explicit approval.
- `TODO.md` tracks active and recent work — keep it terse, factual,
  one-bullet-per-outcome. No marketing language.
- `docs/notes/` holds longer design references and historical decisions.
- After completing work, update `TODO.md` in the same change.

---

## 14. Where Things Live

- **Code by mission**: `src/<module>/`, see Module Map above.
- **Reference apps**: `sample/quickstart/` (smallest possible),
  `sample/pointcloud_viewer/` (file → render with selection),
  `sample/editor/` (vis+editing dogfood check),
  `sample/quickviz_demo_app/` (broader app shell demo).
- **Active TODOs**: `TODO.md` (root).
- **Architecture deep-dives**: `docs/notes/`.
- **Build / install**: `README.md`.
- **CI**: `.github/workflows/default.yml` (note: includes a
  `boundary-check` job that gates the build matrix).

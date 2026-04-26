# QuickViz Project Guidelines

This document provides comprehensive guidance for working with the QuickViz C++ visualization library for robotics applications.

## Project Overview

QuickViz is a C++ visualization library for robotics applications, providing:
- **imview**: Automatic layout management and UI widgets (buttons, sliders, text boxes)
- **gldraw**: 2D/3D real-time rendering with OpenGL
- **widget**: Cairo-based drawing and plotting widgets
- **core**: Event system, buffers, and shared utilities

## Core Development Principles

### Design Philosophy
- **Small surface, strong contracts**: Keep public APIs minimal and explicit; hide implementation details
- **Seams before abstractions**: Extract clear boundaries first; abstract later if duplication persists
- **Single responsibility**: Each module/file does one thing well (target ~500 LOC per file)
- **Local reasoning**: Callers shouldn't need global state knowledge to use an API
- **Performance by design**: Favor data-oriented layouts, predictable memory, and measured hot paths
- **Determinism over cleverness**: Prefer simple, reproducible behavior to smart but fragile logic
- **Building blocks philosophy**: Provide generic, composable components that users can combine to build domain-specific applications. Use generic terms (geometry, mesh, camera, viewport) rather than application-specific terminology (map, terrain, navigation)

### Library Interface Boundaries
QuickViz is designed as a toolkit of building blocks for robotics visualization applications. The interface design clearly distinguishes between different levels of user interaction:

**Direct Use Components** (Ready-to-use, minimal configuration):
- Core rendering primitives (points, lines, meshes, textures)
- Standard UI widgets (buttons, sliders, text inputs)
- Camera controllers and viewport management
- File I/O utilities (image, mesh formats)

**Configurable Components** (Parameters and settings exposed):
- Rendering passes and shaders (lighting models, post-processing)
- Layout managers and containers
- Event handling and input mapping
- Color schemes and visual styling

**Extensible Components** (Virtual interfaces for inheritance):
- `Renderable`, `InputHandler`, `SceneObject` interfaces
- Custom drawing and interaction tools
- Specialized data visualization widgets
- Custom file format adapters

**Build-Upon Components** (Frameworks for complex applications):
- Scene graph architecture
- Command/undo system
- Threading and job systems
- Plugin and extension mechanisms

> **Design Rule**: Generic robotics and graphics terminology should be preferred over domain-specific names. For example, use "Mesh", "PointCloud", "Camera" rather than "Map", "Scan", "Observer". This ensures the library remains broadly applicable across different robotics applications.

### When to Create or Split Modules
Create (or split) a module when:
- **Two or more** other modules depend on a concept
- The code has **distinct lifecycles** (e.g., GPU resources vs. CPU parsing)
- You need to **swap implementations** behind an interface
- You want **separate testability** (unit tests without GL context)

**Do not** create a module if it only wraps 1-2 functions without clear benefit.

## Architecture & Dependencies

### Module Structure
```
src/
├── core/          # Event system, buffers, utilities (depends on nothing)
├── imview/        # GLFW window management, ImGui integration
├── widget/        # Cairo drawing, image widgets, plotting
├── gldraw/        # OpenGL 3D rendering, point clouds, textures
├── pcl_bridge/    # Optional PCL adapter (file loading, conversions)
├── cvdraw/        # OpenCV-based drawing utilities (optional bridge)
└── third_party/   # imgui, implot, stb, yoga, googletest

sample/            # Reference applications built ON TOP of the library
```

### Dependency Rules
- **Core** (math, logging, utilities) depends on nothing else
- **Model** (scene/data types, transforms, selection) may depend on Core
- **Graphics** (GL wrappers, passes, shaders) may depend on Core and Model
- **Tools/Interaction** (picking, gizmos, measure) may depend on Graphics + Model
- **UI** (ImGui panels, docking) can depend on everything but is never depended on
- **Bridges/Adapters** (OpenCV/PCL/etc.) depend outward; nothing core depends on them

> **Rule**: Lower layers never include headers from higher layers

### Library Boundary: `src/` is visualization-only
QuickViz is a visualization library. Editor / app-level concerns
(undo/redo, command history, scene serialization, project files, editing
operations) live in `sample/` and consume the library, never the reverse.

- `sample/*` may include from `src/*/include/` and link against library targets.
- `src/*` must not include from `sample/`. Enforced by CI (`boundary-check`).
- If a sample needs something from the library, add it as an additive,
  visualization-justified hook to the library — do not pull sample code in.
- Sample applications also serve as a dogfood check: if a fully-featured
  vis+editing app cannot be built on top of `src/` without modifying `src/`,
  the library is missing a hook and we evaluate the gap deliberately.

### Key Design Patterns

#### 1. Scene Object Hierarchy (imview)
- `Window` → `Viewer` → `SceneObject`
- `SceneObject` implements: `Renderable`, `Resizable`, `InputHandler`
- `Panel` extends `SceneObject` for ImGui panels
- `Box` provides container with automatic layout via Yoga

#### 2. OpenGL Rendering Pipeline (gldraw)
- `GlSceneManager` manages OpenGL objects and framebuffer
- `OpenGlObject` interface for all renderable 3D objects
- Render-to-texture approach with `FrameBuffer`
- `Camera` + `CameraController` for 3D navigation

#### 3. Multi-Layer Point Cloud System
- `LayerManager` handles multiple rendering layers with priorities
- `PointLayer` for subset rendering with custom colors/sizes
- PCL bridge utilities for integration with Point Cloud Library

## Build System & Dependencies

### Initial Setup
```bash
# Clone with submodules
git clone --recursive https://github.com/rxdu/quickviz.git
git submodule update --init --recursive

# Install dependencies (Ubuntu 22.04/24.04)
sudo apt-get install libgl1-mesa-dev libglfw3-dev libcairo2-dev \
                     libopencv-dev libglm-dev libncurses-dev

# Optional: Install PCL for point cloud features
sudo apt-get install libpcl-dev

# Optional: Development tools
sudo apt-get install valgrind libbenchmark-dev lcov
```

### Build Configuration
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
make -j8
```

### CMake Options
- `BUILD_TESTING`: Enable tests (OFF by default)
- `QUICKVIZ_DEV_MODE`: Development mode, forces tests (OFF by default)
- `ENABLE_AUTO_LAYOUT`: Enable Yoga-based automatic layout (ON by default, requires C++20)
- `BUILD_QUICKVIZ_APP`: Build the quickviz application (OFF by default)
- `IMVIEW_WITH_GLAD`: Integrate GLAD for OpenGL loading (ON by default)
- `STATIC_CHECK`: Enable cppcheck static analysis (OFF by default)

### Dependencies
**Required**: OpenGL 3.3+, GLFW3, GLM, Cairo
**Optional**: OpenCV (cvdraw), PCL (point cloud bridge), Google Benchmark, Valgrind
**Bundled**: Dear ImGui & ImPlot, STB libraries, Yoga, GoogleTest, GLAD

## API Design Standards

### Public API Requirements
- **Explicit inputs**: Functions take `projection`, `view`, sizes, IDs; avoid hidden globals
- **Return handles or IDs**: Use opaque 32-bit IDs for user-visible resources; RAII classes for GL internals
- **Clear ownership**: Prefer `std::unique_ptr` in APIs; only use `std::shared_ptr` for true sharing
- **Narrow types**: Use `span<T const>` for read-only bulk data; avoid exposing STL containers
- **Unit awareness**: Document meters/seconds/radians; never assume degrees or "pixels" for world scale
- **Generic terminology**: Use standard robotics/graphics terms (Geometry, Transform, Viewport) over application-specific names (Map, World, Scene). This ensures broad applicability across different domains

### Interface Design Patterns

#### Extensibility Levels
Design APIs with clear extensibility boundaries:

```cpp
// Direct Use: Simple, concrete functions
void DrawPoints(span<const glm::vec3> points, glm::vec3 color, float size);
void DrawMesh(const MeshData& mesh, const Transform& transform);

// Configurable: Parameter objects for complex configurations  
struct RenderConfig {
  LightingModel lighting = LightingModel::kPhong;
  bool wireframe = false;
  float point_size = 1.0f;
};
void DrawMesh(const MeshData& mesh, const Transform& transform, const RenderConfig& config);

// Extensible: Virtual interfaces for custom behavior
class Renderable {
public:
  virtual void OnRender(const RenderContext& context) = 0;
  virtual BoundingBox GetBounds() const = 0;
};

// Build-Upon: Framework classes with protected extension points
class InteractionTool {
public:
  void HandleInput(const InputEvent& event);  // final
protected:
  virtual bool OnMouseDown(int x, int y) { return false; }  // override points
  virtual bool OnMouseDrag(int x, int y) { return false; }
  virtual void OnToolActivated() {}
};
```

### API Pattern Examples

#### Opaque Handles + Narrow Interface
```cpp
using ObjectId = uint32_t;  // 0 reserved as "none"

struct View {
  glm::mat4 projection;  // float
  glm::mat4 view;        // float
  int width, height;
};

ObjectId CreateMesh(span<const Vertex> vertices, span<const uint32_t> indices);
void     SetTransform(ObjectId id, const glm::dmat4& world_from_object);
void     DrawView(const View& view);
ObjectId PickAt(const View& view, int x, int y);
```

#### Core Interfaces
```cpp
class Renderable {
    virtual bool IsVisible() = 0;
    virtual void OnRender() = 0;
};

class OpenGlObject {
    virtual void OnDraw(const glm::mat4& projection, const glm::mat4& view) = 0;
};
```

## Rendering Pipeline Best Practices

### OpenGL 3.3+ Guidelines
- **Isolate passes**: Each pass sets all required GL state (program, VAO, FBO, depth, blend, cull)
- **Shared camera block**: Put `proj`/`view` in UBO; bind once per pass
- **Per-draw data**: Prefer small UBO/SSBO structs over many `glUniform*` calls
- **Debug output**: Enable `GL_KHR_debug` in debug builds

### GPU Resource Management
- **RAII GL objects**: Thin wrappers for VAO/VBO/EBO/FBO/Program; no naked GLuints
- **Buffer usage**:
   - Static: `glBufferData` or `glBufferStorage`
   - Dynamic: `glMapBufferRange` with appropriate flags
- **Batching**: Sort by program → material → geometry
- **Minimize readbacks**: Only read pixels for picking; never read large buffers per frame

### GL Pass Pattern
```cpp
class Pass {
 public:
  void Execute(const View& view) {
    BindFbo();
    ConfigureState();     // depth/blend/cull
    UseProgram();         // bind UBOs/textures
    DrawAll();            // VAO binds and draw calls
  }
 private:
  void BindFbo();
  void ConfigureState();
  void UseProgram();
  void DrawAll();
};
```

## Coordinate System & Precision

- **CPU double, GPU float**: Keep CPU transforms in `double`; upload as `float` to shaders
- **Consistent "up" direction**: Support Z-up or Y-up at boundary; convert once internally
- **Camera sanity**: Warn if far/near > 1e6 to avoid z-fighting

## Point Cloud Enhancement Features

### PCL Integration
- Import/export between PCL and renderer formats
- Visualization of PCL algorithm results (clusters, surfaces)
- Template-based conversions for all PCL point types

### Multi-Layer Rendering System
**Core Features**:
- Priority-based layer composition (higher priority renders on top)
- Multiple highlight modes: surface fill, outline, size increase
- Index buffer optimization for efficient batch rendering (60-100x improvement)
- 3D sphere rendering with Phong lighting

**Usage Example**:
```cpp
// Create selection layer
auto selection_layer = point_cloud->CreateLayer("selection", 100);
selection_layer->SetPoints(selected_indices);
selection_layer->SetColor(glm::vec3(1.0f, 1.0f, 0.0f)); // Yellow
selection_layer->SetPointSizeMultiplier(1.5f);
selection_layer->SetHighlightMode(PointLayer::HighlightMode::kSphereSurface);
selection_layer->SetVisible(true);
```

## Interaction & Editing Patterns

### Two-Stage Picking
1. **GPU ID buffer** → object ID
2. **CPU raycast** → precise hit/feature (BVH/KD-tree)

### Tool State Machine
- One active tool at a time
- Tools consume input and emit **commands** (for undo/redo)
- Gizmos drawn as overlay with selective depth testing

### Command Pattern (Undo/Redo)
```cpp
class Command {
 public:
  virtual ~Command() = default;
  virtual void Do() = 0;
  virtual void Undo() = 0;
};

class CommandStack {
 public:
  void Exec(std::unique_ptr<Command> cmd);
  bool CanUndo() const;
  bool CanRedo() const;
  void Undo();
  void Redo();
 private:
  std::vector<std::unique_ptr<Command>> done_, undone_;
};
```

## Threading Model

- **GL main-thread only**: All GL calls and ImGui rendering on render thread
- **Background stages**: File I/O, decoding, normal generation, BVH builds
- **Handoff boundary**: Jobs produce immutable CPU buffers; enqueue main-thread task for GL resources
- **No frame stalls**: Never wait on background jobs in frame loop

## Code Quality Standards

### Style Guide
- **C++ Standard**: C++17 (C++14 minimum for Ubuntu 20.04)
- **File Extensions**: `.cpp` for source, `.hpp` for headers
- **Style**: Google C++ style guide, clang-format
- **Line Length**: 100 characters

### Testing Strategy
- **Unit Tests**: Core functionality (`tests/unit/`)
- **Integration Tests**: Module interactions (`tests/integration/`)
- **Memory Tests**: Leak detection (`tests/memory/`)
- **Benchmarks**: Performance testing (`tests/benchmarks/`)

### Test Execution
```bash
# Basic tests
ctest --output-on-failure

# Comprehensive test suite
../scripts/run_tests.sh

# With Valgrind and coverage
../scripts/run_tests.sh -v -c
```

### Performance Hygiene
- Avoid per-frame heap churn: pre-allocate vectors, reuse temporaries
- Prefer SoA (structure-of-arrays) for large numeric datasets
- Keep shader branches simple; use separate passes for complex modes
- Profile both CPU and GPU; log slowest draw calls in debug

## Error Handling & Robustness

- **Structured logs**: Include frame number, pass name, object ID
- **Graceful degradation**: Render fallback (magenta material) on shader/asset failure
- **Debug assertions**: `DCHECK` invariants in debug builds only
- **GL debug context**: Enable in debug builds; treat high severity as test failures

## External Dependencies & Bridges

- Put adapters to heavy dependencies behind **small interfaces**
- Keep core build working when optional dependencies are **absent**
- No upstream types in public headers (don't leak `pcl::PointXYZ`)

## Development Workflow

### Feature Development
1. Create feature branch from `devel`
2. Implement with tests using established patterns
3. Run `scripts/run_tests.sh` before committing
4. PR to `devel` for review

### Common Tasks

**Add New Renderable Object**:
1. Inherit from `OpenGlObject` in `src/gldraw/renderable/`
2. Implement shader loading and VAO/VBO setup
3. Override `OnDraw()` with OpenGL render calls
4. Add to `GlSceneManager` in application

**Create Custom UI Panel**:
1. Inherit from `Panel` in `src/imview/`
2. Override `Begin()` and `End()` methods with ImGui calls
3. Add panel to `Viewer` or `Box` container

### Refactor Playbook
1. **Define boundary** (what belongs inside vs. outside)
2. **Write tiny interface** (2-5 functions, minimal templates)
3. **Add facade** that forwards to existing code
4. **Add tests** around facade
5. **Move code** under facade into new module
6. **Delete old paths** once coverage passes
7. **Measure performance** to ensure no regression

## Decision Heuristics

- **Expose or hide?** If type couples callers to OpenGL/third-party, **hide** it
- **Template or runtime?** If callers won't benefit from compile-time polymorphism, **prefer runtime**
- **One pass or two?** If branch toggles many states, **split into passes**
- **CPU or GPU?** Precision/topology → **CPU** (BVH/KD); per-pixel labeling → **GPU** (ID buffer)
- **Immediate or queued?** GPU resource allocation should be **queued** to render thread
- **Generic or specific?** Prefer generic robotics/graphics terms over application-specific names to maximize reusability
- **Direct use or extensible?** Simple, common operations should be directly callable; complex customization should use virtual interfaces
- **Framework or library?** Provide building blocks that users compose rather than frameworks that dictate application structure

## PR Review Checklist

- [ ] No GL calls off render thread
- [ ] Functions have explicit inputs; no new hidden globals
- [ ] Render paths set depth/blend/cull/program/VAO/FBO explicitly
- [ ] CPU uses double for geometry; GPU uniforms are float
- [ ] Picking reads exactly one pixel; ray logic has tests
- [ ] No raw GL handles in public headers
- [ ] No per-frame allocations on hot paths
- [ ] clang-format/clang-tidy clean; zero new warnings
- [ ] Documentation for non-obvious decisions
- [ ] Generic terminology used (avoid application-specific names)
- [ ] Interface boundaries clearly defined (direct use vs. extensible vs. build-upon)
- [ ] Building blocks remain composable and reusable across different applications

## Development Rules

- Remember to update TODO.md after getting approval for new tasks
- Always update TODO.md after finishing tasks
- Document architectural decisions with brief "Why this way?" notes
- Maintain backwards compatibility within major versions
- Prefer measured optimization over premature optimization

## Platform Support

- **Linux**: Primary platform (Ubuntu 20.04/22.04/24.04)
- **Windows**: Experimental via vcpkg
- **macOS**: Not officially supported

## Current Development Focus

Active areas of development:
- Interactive selection tools
- PCL algorithm result visualization
- Measurement and annotation overlays
- Level-of-detail system for large datasets

See `TODO.md` for detailed roadmap and implementation status.
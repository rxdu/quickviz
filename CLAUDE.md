# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

QuickViz is a C++ visualization library for robotics applications, providing:
- **imview**: Automatic layout management and UI widgets (buttons, sliders, text boxes)
- **renderer**: 2D/3D real-time rendering with OpenGL
- **widget**: Cairo-based drawing and plotting widgets
- **core**: Event system, buffers, and shared utilities

## Build Commands

### Initial Setup
```bash
# Clone with submodules
git clone --recursive https://github.com/rxdu/quickviz.git
# Or update submodules after cloning
git submodule update --init --recursive

# Install dependencies (Ubuntu 22.04/24.04)
sudo apt-get install libgl1-mesa-dev libglfw3-dev libcairo2-dev libopencv-dev libglm-dev libncurses-dev

# Optional: Install PCL for point cloud features
sudo apt-get install libpcl-dev

# Optional: Development tools
sudo apt-get install valgrind libbenchmark-dev lcov
```

### Build Project
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
make -j8
```

### Run Tests
```bash
# Basic tests
ctest --output-on-failure

# Comprehensive test suite
../scripts/run_tests.sh

# With Valgrind memory checking
../scripts/run_tests.sh -v

# With code coverage
../scripts/run_tests.sh -c

# With benchmarks
../scripts/run_tests.sh -b
```

### CMake Options
- `BUILD_TESTING`: Enable tests (OFF by default)
- `QUICKVIZ_DEV_MODE`: Development mode, forces tests (OFF by default)
- `ENABLE_AUTO_LAYOUT`: Enable Yoga-based automatic layout (ON by default, requires C++20)
- `BUILD_QUICKVIZ_APP`: Build the quickviz application (OFF by default)
- `IMVIEW_WITH_GLAD`: Integrate GLAD for OpenGL loading (ON by default)
- `STATIC_CHECK`: Enable cppcheck static analysis (OFF by default)

## Architecture

### Module Structure
```
src/
├── core/          # Event system, buffers, utilities
├── imview/        # GLFW window management, ImGui integration
├── widget/        # Cairo drawing, image widgets, plotting
├── renderer/      # OpenGL 3D rendering, point clouds, textures
├── cvdraw/        # OpenCV-based drawing utilities (optional)
└── third_party/   # imgui, implot, stb, yoga, googletest
```

### Key Design Patterns

#### 1. Scene Object Hierarchy (imview)
- `Window` → `Viewer` → `SceneObject`
- `SceneObject` implements: `Renderable`, `Resizable`, `InputHandler`
- `Panel` extends `SceneObject` for ImGui panels
- `Box` provides container with automatic layout via Yoga

#### 2. OpenGL Rendering Pipeline (renderer)
- `GlSceneManager` manages OpenGL objects and framebuffer
- `OpenGlObject` interface for all renderable 3D objects
- Render-to-texture approach with `FrameBuffer`
- `Camera` + `CameraController` for 3D navigation

#### 3. Multi-Layer Point Cloud System
- `LayerManager` handles multiple rendering layers with priorities
- `PointLayer` for subset rendering with custom colors/sizes
- PCL bridge utilities for integration with Point Cloud Library

### Core Interfaces

#### Renderable
```cpp
class Renderable {
    virtual bool IsVisible() = 0;
    virtual void OnRender() = 0;
};
```

#### OpenGlObject
```cpp
class OpenGlObject {
    virtual void OnDraw(const glm::mat4& projection, const glm::mat4& view) = 0;
};
```

## Point Cloud Enhancement Features

### PCL Integration (when PCL is available)
- Import/export between PCL and renderer formats
- Visualization of PCL algorithm results (clusters, surfaces)
- Template-based conversions for all PCL point types

### Layer System
- Multiple rendering layers with priority-based composition
- Per-layer highlighting, selection, and visual effects
- Blend modes and opacity controls

## Code Style

- **C++ Standard**: C++17 (C++14 minimum for Ubuntu 20.04)
- **File Extensions**: `.cpp` for source, `.hpp` for headers
- **Style Guide**: Google C++ style guide
- **Line Length**: 100 characters
- **Formatting**: clang-format with Google style

## Testing Strategy

### Test Categories
- **Unit Tests**: Core functionality (`tests/unit/`)
- **Integration Tests**: Module interactions (`tests/integration/`)
- **Memory Tests**: Leak detection (`tests/memory/`)
- **Benchmarks**: Performance testing (`tests/benchmarks/`)

### Test Labels (for ctest -L)
- `unit`: Unit tests
- `integration`: Integration tests
- `memory`: Memory leak tests
- `valgrind`: Valgrind-specific tests

## Development Workflow

1. **Feature Development**:
   - Create feature branch from `devel`
   - Implement with tests
   - Run `scripts/run_tests.sh` before committing
   - PR to `devel` for review

2. **Adding New OpenGL Objects**:
   - Inherit from `OpenGlObject` interface
   - Implement `OnDraw()` method
   - Add to `GlSceneManager` for rendering

3. **Adding UI Panels**:
   - Inherit from `Panel` class
   - Override `Begin()` and `End()` for ImGui content
   - Use `SetAutoLayout(true)` for automatic positioning

## Common Tasks

### Add a New Renderable Object
1. Create class inheriting from `OpenGlObject` in `src/renderer/renderable/`
2. Implement shader loading and VAO/VBO setup
3. Override `OnDraw()` with OpenGL render calls
4. Add to `GlSceneManager` in application code

### Create a Custom UI Panel
1. Inherit from `Panel` in `src/imview/`
2. Override `Begin()` and `End()` methods
3. Add ImGui calls between Begin/End
4. Add panel to `Viewer` or `Box` container

### Integrate PCL Algorithm Results
1. Use `pcl_bridge::ImportFromPCL()` to convert PCL cloud
2. Process with PCL algorithms
3. Use `pcl_bridge::VisualizePCLResults()` for display
4. Add layers for highlighting results

## Dependencies

### Required
- OpenGL 3.3+
- GLFW3
- GLM
- Cairo

### Optional
- OpenCV (for cvdraw module)
- PCL (for point cloud bridge utilities)
- Google Benchmark (for performance tests)
- Valgrind (for memory tests)

### Bundled (in third_party/)
- Dear ImGui & ImPlot
- STB libraries
- Yoga (layout engine)
- GoogleTest
- GLAD (OpenGL loader)

## Platform Support

- **Linux**: Primary platform (Ubuntu 20.04/22.04/24.04)
- **Windows**: Experimental via vcpkg
- **macOS**: Not officially supported

## Current Development Focus

Active development on point cloud visualization enhancements:
- Interactive selection tools (in progress)
- PCL algorithm result visualization
- Measurement and annotation overlays
- Level-of-detail system for large datasets

See `TODO.md` for detailed roadmap and implementation status. 

## Development Rules

* Remember to update TODO.md after getting approval for a new task.
* Always update TODO.md after you finish a task.

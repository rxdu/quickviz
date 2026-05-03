# sample/quickstart — the smallest QuickViz program

This sample exists to prove that QuickViz lets you build a working
visualization tool in roughly thirty lines of code. If you just cloned the
repo and want to know "how do I display *something*", read `main.cpp` —
that's the answer.

## What it does

Opens a window, draws a colored helix made of 2000 points in 3D space, lets
you orbit the camera with the mouse. Closes when you close the window.

## The whole program (with annotations)

```cpp
#include "core/demo.hpp"                   // synthetic data generators
#include "scene/renderable/point_cloud.hpp"
#include "scene/scene_app.hpp"             // 5-line quickstart facade

int main() {
  quickviz::SceneApp::Config config;       // sensible defaults: grid + axes on
  config.window_title = "QuickViz Quickstart";
  quickviz::SceneApp app(config);

  app.SetSceneSetup([](quickviz::SceneManager* scene) {
    // SceneSetup runs once after the OpenGL context is ready.
    auto data = quickviz::demo::SpiralCloud(2000);   // synthetic points
    auto cloud = std::make_unique<quickviz::PointCloud>();
    cloud->SetPointSize(4.0f);
    cloud->SetPoints(data.points, data.colors);
    scene->AddOpenGLObject("spiral", std::move(cloud));
  });

  app.Run();   // blocks until window is closed
}
```

That's it. No layout boilerplate, no manual ImGui setup, no GLFW glue, no
shaders to write.

## What you can change without learning more

- Swap `SpiralCloud` for `PlanarPointGrid(50, 50, 0.1f)` or
  `NoiseCloud(5000, 1.5f)` — see `core/demo.hpp` for the full list.
- Add a `Mesh` instead of (or alongside) the cloud:

  ```cpp
  auto mesh_data = quickviz::demo::CubeMesh(glm::vec3{0, 0, 0}, 1.5f);
  auto mesh = std::make_unique<quickviz::Mesh>();
  mesh->SetVertices(mesh_data.vertices);
  mesh->SetIndices(mesh_data.indices);
  scene->AddOpenGLObject("cube", std::move(mesh));
  ```
- Toggle the reference grid or coordinate frame via
  `config.show_grid = false;` etc.

## When to drop down from `SceneApp`

`SceneApp` is the path of least resistance, not the only door. Reach for
`Viewer` + `Panel` directly when you need:

- Multiple panels (3D + plot + image, tabbed layouts, dockable windows)
- Custom UI panels alongside the scene
- A history / tools / properties sidebar

Look at `sample/pointcloud_viewer/` and `sample/editor/` for examples that
compose `Viewer` directly.

## Building & running

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
./build/bin/quickviz_quickstart
```

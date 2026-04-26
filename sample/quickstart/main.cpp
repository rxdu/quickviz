/*
 * @file main.cpp
 * @brief The smallest possible QuickViz program — proves SceneApp delivers
 *        on its "5-line quickstart" promise. ~25 lines of meaningful code.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <memory>

#include "core/demo.hpp"
#include "scene/renderable/point_cloud.hpp"
#include "scene/scene_app.hpp"

int main() {
  quickviz::SceneApp::Config config;
  config.window_title = "QuickViz Quickstart";
  quickviz::SceneApp app(config);

  app.SetSceneSetup([](quickviz::SceneManager* scene) {
    auto data = quickviz::demo::SpiralCloud(/*num_points=*/2000);
    auto cloud = std::make_unique<quickviz::PointCloud>();
    cloud->SetPointSize(4.0f);
    cloud->SetPoints(data.points, data.colors);
    scene->AddOpenGLObject("spiral", std::move(cloud));
  });

  app.Run();
  return 0;
}

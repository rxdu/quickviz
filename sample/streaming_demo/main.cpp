/*
 * @file main.cpp
 * @brief Canonical streaming pattern: background producer → DataStream →
 *        render thread → renderable.
 *
 * Demonstrates the recommended way to bring sensor-rate data into a
 * QuickViz scene without blocking the render loop. Read this file before
 * writing your first ROS2 / sensor callback — the same shape applies.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "core/data_stream.hpp"
#include "core/demo.hpp"
#include "scene/renderable/point_cloud.hpp"
#include "scene/scene_app.hpp"

int main() {
  using namespace quickviz;

  // The single shared channel between producer and renderer.
  // Latest-only / lossy: if the producer outpaces the renderer, the
  // intermediate clouds are silently dropped — usually what you want.
  DataStream<demo::PointCloudData> stream;

  // Producer thread: simulates a 30 Hz sensor producing a spiral cloud
  // that rotates around the Z axis. In a real app this is your driver
  // callback or ROS2 subscriber.
  std::atomic<bool> stop{false};
  std::thread producer([&]() {
    const auto t0 = std::chrono::steady_clock::now();
    while (!stop.load()) {
      const float t = std::chrono::duration<float>(
          std::chrono::steady_clock::now() - t0).count();
      const float angle = 0.5f * t;  // rad/s
      const float ca = std::cos(angle);
      const float sa = std::sin(angle);

      auto data = demo::SpiralCloud(2000);
      for (auto& p : data.points) {
        const float x = p.x * ca - p.y * sa;
        const float y = p.x * sa + p.y * ca;
        p.x = x;
        p.y = y;
      }
      stream.Push(std::move(data));

      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  });

  SceneApp::Config config;
  config.window_title = "QuickViz Streaming Demo";
  SceneApp app(config);

  // Wire up the scene: a single empty point cloud, plus a pre-draw
  // callback that drains the stream once per frame.
  PointCloud* cloud = nullptr;
  app.SetSceneSetup([&](SceneManager* scene) {
    auto pc = std::make_unique<PointCloud>();
    pc->SetPointSize(4.0f);
    cloud = pc.get();
    scene->AddOpenGLObject("stream", std::move(pc));

    // Pre-draw callback runs on the render thread, just before the
    // scene is rendered. Non-blocking; if no fresh value is available,
    // the previous frame's cloud is rendered again.
    scene->SetPreDrawCallback([&]() {
      demo::PointCloudData latest;
      if (stream.TryPull(latest)) {
        cloud->SetPoints(latest.points, latest.colors);
      }
    });
  });

  app.Run();

  // Clean shutdown: stop the producer before the stream goes out of
  // scope. Order matters — the producer holds a reference to `stream`.
  stop.store(true);
  producer.join();
  return 0;
}

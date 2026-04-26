/*
 * @file main.cpp
 * @brief sample/editor — vis+editing reference app on top of quickviz
 *
 * This sample exists as the dogfood check on the library: it must be
 * implementable without modifying anything under src/. If you find yourself
 * tempted to reach into src/ from here, that's a signal the library is
 * missing a visualization-justified hook — log it and we evaluate.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "pcl_bridge/pcl_loader.hpp"

#include "editor_state.hpp"
#include "editor_viewport.hpp"
#include "panels/editor_tool_panel.hpp"
#include "panels/history_panel.hpp"

using namespace quickviz;

namespace {

struct LoadedCloud {
  std::vector<glm::vec3> points;
  std::vector<glm::vec3> colors;
  glm::vec3 min_bounds{0.0f};
  glm::vec3 max_bounds{0.0f};
};

LoadedCloud LoadFromFile(const std::string& path) {
  auto [renderer_data, metadata] =
      pcl_bridge::factory::FactoryRegistry::LoadForRenderer(
          path, pcl_bridge::PointCloudLoader::Format::kAutoDetect);

  LoadedCloud out;
  out.min_bounds = metadata.min_bounds;
  out.max_bounds = metadata.max_bounds;

  using ColorMode = pcl_bridge::factory::RendererData::ColorMode;
  if (renderer_data.color_mode == ColorMode::kRGB) {
    out.points = std::move(renderer_data.points_3d);
    out.colors = std::move(renderer_data.colors_rgb);
  } else {
    // Build a simple height-mapped color so non-RGB clouds remain editable.
    out.points.reserve(renderer_data.points_4d.size());
    out.colors.reserve(renderer_data.points_4d.size());
    const float zmin = metadata.min_bounds.z;
    const float zmax = metadata.max_bounds.z;
    const float range = std::max(1e-6f, zmax - zmin);
    for (const auto& p4 : renderer_data.points_4d) {
      out.points.emplace_back(p4.x, p4.y, p4.z);
      const float t = (p4.z - zmin) / range;
      out.colors.emplace_back(t, 1.0f - t, 0.5f);
    }
  }
  return out;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path-to-pcd-or-ply>\n";
    return EXIT_FAILURE;
  }
  const std::string cloud_path = argv[1];

  try {
    LoadedCloud loaded = LoadFromFile(cloud_path);
    std::cout << "Loaded " << loaded.points.size() << " points from "
              << cloud_path << std::endl;

    Viewer viewer("QuickViz Editor", 1400, 900);

    auto state = std::make_shared<editor::EditorState>();

    auto viewport =
        std::make_shared<editor::EditorViewport>("Editor", state.get());
    viewport->SetAutoLayout(true);
    viewport->SetNoTitleBar(false);
    viewport->SetFlexGrow(0.78f);
    viewport->SetFlexShrink(1.0f);

    auto cloud = std::make_unique<PointCloud>();
    cloud->SetPointSize(4.0f);
    cloud->SetOpacity(1.0f);
    cloud->SetRenderMode(PointMode::kPoint);
    cloud->SetPoints(loaded.points, loaded.colors);
    viewport->AddOpenGLObject(editor::kEditableCloudName, std::move(cloud));

    auto* cloud_ptr = dynamic_cast<PointCloud*>(
        viewport->GetOpenGLObject(editor::kEditableCloudName));
    if (!cloud_ptr) {
      std::cerr << "Failed to register editable point cloud\n";
      return EXIT_FAILURE;
    }

    // Reference grid (non-editable visual aid).
    const glm::vec3 size = loaded.max_bounds - loaded.min_bounds;
    const float extent = std::max({size.x, size.y, 1.0f});
    auto grid = std::make_unique<Grid>(extent * 0.1f, extent,
                                       glm::vec3(0.5f, 0.5f, 0.5f));
    viewport->AddOpenGLObject("reference_grid", std::move(grid));

    state->Initialize(cloud_ptr, viewport->GetTool(), std::move(loaded.points),
                      std::move(loaded.colors));

    auto tool_panel =
        std::make_shared<editor::EditorToolPanel>("Editor Tools", state.get());
    tool_panel->SetAutoLayout(true);
    tool_panel->SetNoTitleBar(false);
    tool_panel->SetFlexGrow(0.45f);
    tool_panel->SetFlexShrink(0.0f);

    auto history_panel =
        std::make_shared<editor::HistoryPanel>("History", state.get());
    history_panel->SetAutoLayout(true);
    history_panel->SetNoTitleBar(false);
    history_panel->SetFlexGrow(0.55f);
    history_panel->SetFlexShrink(0.0f);

    auto side_box = std::make_shared<Box>("editor_side_box");
    side_box->SetFlexDirection(Styling::FlexDirection::kColumn);
    side_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    side_box->SetAlignItems(Styling::AlignItems::kStretch);
    side_box->SetFlexGrow(0.22f);
    side_box->SetFlexShrink(0.0f);
    side_box->AddChild(tool_panel);
    side_box->AddChild(history_panel);

    auto main_box = std::make_shared<Box>("editor_main_box");
    main_box->SetFlexDirection(Styling::FlexDirection::kRow);
    main_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    main_box->SetAlignItems(Styling::AlignItems::kStretch);
    main_box->AddChild(viewport);
    main_box->AddChild(side_box);

    viewer.AddSceneObject(main_box);
    viewer.Show();

  } catch (const pcl_bridge::FileNotFoundException& e) {
    std::cerr << "File not found: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (const pcl_bridge::UnsupportedFormatException& e) {
    std::cerr << "Unsupported format: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

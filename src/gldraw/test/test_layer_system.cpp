/*
 * test_layer_system.cpp
 *
 * Created on: Dec 2024
 * Description: Test the multi-layer rendering system for point cloud highlighting
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <random>
#include <glm/glm.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/layer_manager.hpp"

using namespace quickviz;

void TestBasicLayering() {
    std::cout << "=== Testing Basic Layer Management ===" << std::endl;
    
    // Create test point cloud data
    std::vector<glm::vec4> test_points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dist(-10.0f, 10.0f);
    std::uniform_real_distribution<float> intensity_dist(0.0f, 1.0f);
    
    // Create 500 random points
    for (int i = 0; i < 500; ++i) {
        test_points.push_back(glm::vec4(
            pos_dist(gen), pos_dist(gen), pos_dist(gen), intensity_dist(gen)
        ));
    }
    
    std::cout << "Created " << test_points.size() << " test points" << std::endl;
    
    // Create viewer and scene
    Viewer viewer;
    auto box = std::make_shared<Box>("layer_test_box");
    box->SetFlexDirection(Styling::FlexDirection::kRow);
    
    auto gl_sm = std::make_shared<GlSceneManager>("Layer System Test");
    gl_sm->SetAutoLayout(true);
    gl_sm->SetNoTitleBar(true);
    gl_sm->SetFlexGrow(1.0f);
    
    // Create point cloud and add test data
    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPoints(test_points, PointCloud::ColorMode::kScalarField);
    point_cloud->SetScalarRange(0.0f, 1.0f);
    point_cloud->SetPointSize(6.0f);
    point_cloud->SetRenderMode(PointMode::kPoint);
    
    // Test layer management
    std::cout << "\n--- Testing Layer Creation ---" << std::endl;
    auto highlight_layer = point_cloud->CreateLayer("highlights", 100);
    auto selection_layer = point_cloud->CreateLayer("selection", 200);
    auto cluster_layer = point_cloud->CreateLayer("clusters", 50);
    
    std::cout << "✓ Created 3 layers" << std::endl;
    
    // Highlight some random points (red)
    std::vector<size_t> highlight_indices;
    std::uniform_int_distribution<size_t> idx_dist(0, test_points.size() - 1);
    for (int i = 0; i < 50; ++i) {
        highlight_indices.push_back(idx_dist(gen));
    }
    point_cloud->HighlightPoints(highlight_indices, glm::vec3(1.0f, 0.0f, 0.0f), "highlights", 2.0f);
    std::cout << "✓ Highlighted " << highlight_indices.size() << " points in red" << std::endl;
    
    // Select some points (yellow)
    std::vector<size_t> selection_indices;
    for (int i = 0; i < 30; ++i) {
        selection_indices.push_back(idx_dist(gen));
    }
    point_cloud->SetSelectedPoints(selection_indices, glm::vec3(1.0f, 1.0f, 0.0f));
    std::cout << "✓ Selected " << selection_indices.size() << " points in yellow" << std::endl;
    
    // Create cluster visualization (green)
    std::vector<size_t> cluster_indices;
    for (int i = 0; i < 80; ++i) {
        cluster_indices.push_back(idx_dist(gen));
    }
    auto cluster_layer_ptr = point_cloud->GetLayer("clusters");
    if (cluster_layer_ptr) {
        cluster_layer_ptr->SetPoints(cluster_indices);
        cluster_layer_ptr->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        cluster_layer_ptr->SetPointSizeMultiplier(1.5f);
        cluster_layer_ptr->SetHighlightMode(PointLayer::HighlightMode::kColorAndSize);
    }
    std::cout << "✓ Created cluster visualization with " << cluster_indices.size() << " points in green" << std::endl;
    
    // Print layer statistics
    point_cloud->GetLayerManager().PrintLayerInfo();
    
    // Add grid for reference
    auto grid = std::make_unique<Grid>(20.0f, 2.0f, glm::vec3(0.5f, 0.5f, 0.5f));
    
    gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
    gl_sm->AddOpenGLObject("grid", std::move(grid));
    
    box->AddChild(gl_sm);
    viewer.AddSceneObject(box);
    
    std::cout << "\n=== Layer System Test Controls ===" << std::endl;
    std::cout << "Base point cloud: Blue-to-red intensity coloring" << std::endl;
    std::cout << "Red highlights: Random points with size multiplier 2.0x (Priority 100)" << std::endl;
    std::cout << "Yellow selection: Selected points with size multiplier 1.8x (Priority 200)" << std::endl;
    std::cout << "Green clusters: Cluster points with size multiplier 1.5x (Priority 50)" << std::endl;
    std::cout << "Higher priority layers render on top" << std::endl;
    
    viewer.Show();
}

void TestLayerManagerStandalone() {
    std::cout << "\n=== Testing LayerManager Standalone ===" << std::endl;
    
    LayerManager manager;
    
    // Test layer creation and management
    auto layer1 = manager.CreateLayer("layer1", 10);
    auto layer2 = manager.CreateLayer("layer2", 20);
    auto layer3 = manager.CreateLayer("layer3", 5);
    
    std::cout << "✓ Created 3 layers" << std::endl;
    
    // Add points to layers
    layer1->SetPoints({0, 1, 2, 3, 4});
    layer1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    
    layer2->SetPoints({2, 3, 4, 5, 6});
    layer2->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    
    layer3->SetPoints({4, 5, 6, 7, 8});
    layer3->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    
    std::cout << "✓ Added points to layers" << std::endl;
    
    // Test layer queries
    auto layers_with_point_4 = manager.GetLayersContainingPoint(4);
    std::cout << "✓ Point 4 is in " << layers_with_point_4.size() << " layers: ";
    for (const auto& name : layers_with_point_4) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    
    auto top_layer = manager.GetTopLayerContainingPoint(4);
    std::cout << "✓ Top layer containing point 4: " << (top_layer ? top_layer->GetName() : "none") << std::endl;
    
    // Test layer statistics
    auto stats = manager.GetStatistics();
    std::cout << "✓ Layer Statistics:" << std::endl;
    std::cout << "  Total layers: " << stats.total_layers << std::endl;
    std::cout << "  Visible layers: " << stats.visible_layers << std::endl;
    std::cout << "  Total points in layers: " << stats.total_points_in_layers << std::endl;
    std::cout << "  Unique points in layers: " << stats.unique_points_in_layers << std::endl;
    
    // Test layer ordering
    auto ordered_layers = manager.GetLayersByPriority();
    std::cout << "✓ Layers by priority: ";
    for (const auto& layer : ordered_layers) {
        std::cout << layer->GetName() << "(" << layer->GetPriority() << ") ";
    }
    std::cout << std::endl;
    
    // Test render data generation
    auto render_data = manager.GenerateRenderData();
    std::cout << "✓ Generated " << render_data.size() << " render data entries" << std::endl;
    
    manager.PrintLayerInfo();
}

int main(int argc, char* argv[]) {
    std::cout << "QuickViz Layer System Test Suite" << std::endl;
    std::cout << "================================" << std::endl;
    
    if (argc > 1) {
        std::string test_mode = argv[1];
        
        if (test_mode == "visual") {
            TestBasicLayering();
        } else if (test_mode == "manager") {
            TestLayerManagerStandalone();
        } else {
            std::cout << "Unknown test mode: " << test_mode << std::endl;
            std::cout << "Available modes: visual, manager" << std::endl;
            return 1;
        }
    } else {
        // Run both tests by default
        TestLayerManagerStandalone();
        TestBasicLayering();
    }
    
    return 0;
}
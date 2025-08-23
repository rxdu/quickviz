/*
 * @file test_selection_visualizer_demo.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Integration test demonstrating SelectionVisualizer::CreateHighlight()
 *
 * This test loads a point cloud and demonstrates external selection visualization
 * using the new visualization module API. It shows the clean separation between
 * processing (selection algorithms) and visualization.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <random>
#include <algorithm>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/grid.hpp"

// New visualization module includes
#include "visualization/helpers/selection_visualizer.hpp"
#include "visualization/contracts/selection_data.hpp"

#ifdef QUICKVIZ_WITH_PCL
#include "visualization/pcl_bridge/pcl_loader.hpp"
#endif

using namespace quickviz;

class SelectionVisualizerDemo {
public:
    SelectionVisualizerDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager
        scene_manager_ = std::make_shared<GlSceneManager>("Selection Visualizer Demo");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateTestScene() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // Create test point cloud or load from file
        cloud_ = std::make_unique<PointCloud>();
        
#ifdef QUICKVIZ_WITH_PCL
        // Try to load a PCD file if available
        if (TryLoadPCDFile()) {
            std::cout << "Loaded point cloud from PCD file" << std::endl;
        } else {
            CreateSyntheticPointCloud();
        }
#else
        CreateSyntheticPointCloud();
#endif
        
        cloud_ptr_ = cloud_.get();
        scene_manager_->AddOpenGLObject("cloud", std::move(cloud_));
        
        // Demonstrate external selection processing
        DemonstrateSelectionVisualization();
    }
    
    void CreateSyntheticPointCloud() {
        std::vector<glm::vec3> points;
        std::vector<glm::vec3> colors;
        
        // Create a simple 3D structure
        std::mt19937 gen(42);
        std::uniform_real_distribution<float> dis(-5.0f, 5.0f);
        std::uniform_real_distribution<float> color_dis(0.3f, 0.8f);
        
        // Generate 5000 random points
        for (int i = 0; i < 5000; ++i) {
            points.emplace_back(dis(gen), dis(gen), dis(gen));
            colors.emplace_back(color_dis(gen), color_dis(gen), color_dis(gen));
        }
        
        // Add some structured elements
        // Horizontal plane
        for (int x = -10; x <= 10; x += 2) {
            for (int z = -10; z <= 10; z += 2) {
                points.emplace_back(x * 0.2f, -2.0f, z * 0.2f);
                colors.emplace_back(0.2f, 0.8f, 0.2f);
            }
        }
        
        // Vertical line
        for (int y = -10; y <= 10; y += 1) {
            points.emplace_back(2.0f, y * 0.2f, 1.0f);
            colors.emplace_back(0.8f, 0.2f, 0.2f);
        }
        
        cloud_->SetPoints(points, colors);
        cloud_->SetPointSize(2.0f);
        
        std::cout << "Created synthetic point cloud with " << points.size() << " points" << std::endl;
    }
    
#ifdef QUICKVIZ_WITH_PCL
    bool TryLoadPCDFile() {
        // Try common PCD file locations
        std::vector<std::string> pcd_paths = {
            "../data/test_cloud.pcd",
            "../../data/test_cloud.pcd", 
            "/tmp/test_cloud.pcd",
            "test_cloud.pcd"
        };
        
        for (const auto& path : pcd_paths) {
            try {
                auto loader_result = visualization::pcl_bridge::PCDLoader::Load(path);
                if (loader_result.success && !loader_result.points_3d.empty()) {
                    cloud_->SetPoints(loader_result.points_3d, loader_result.colors_rgb);
                    cloud_->SetPointSize(2.0f);
                    std::cout << "Loaded PCD file: " << path << " (" 
                              << loader_result.points_3d.size() << " points)" << std::endl;
                    return true;
                }
            } catch (const std::exception& e) {
                // Continue trying other paths
            }
        }
        return false;
    }
#endif
    
    void DemonstrateSelectionVisualization() {
        if (!cloud_ptr_ || cloud_ptr_->GetPointCount() == 0) {
            std::cerr << "No point cloud available for selection" << std::endl;
            return;
        }
        
        std::cout << "\n=== Demonstrating External Selection Processing ===" << std::endl;
        
        // Simulate external processing algorithms
        
        // 1. Random selection (simulating clustering result)
        auto random_selection = SimulateRandomSelection(200);
        visualization::SelectionData cluster_data;
        cluster_data.selection_name = "cluster_1";
        cluster_data.point_indices = random_selection;
        cluster_data.highlight_color = glm::vec3(1.0f, 0.0f, 0.0f);  // Red
        cluster_data.size_multiplier = 2.0f;
        cluster_data.show_bounding_box = true;
        cluster_data.description = "Detected cluster (simulated)";
        
        bool success = visualization::SelectionVisualizer::CreateHighlight(cluster_data, *cloud_ptr_);
        if (success) {
            std::cout << "✓ Created cluster highlight with " << random_selection.size() << " points" << std::endl;
        }
        
        // 2. Geometric selection (simulating plane detection)
        auto plane_selection = SimulatePlaneDetection();
        visualization::SelectionData plane_data;
        plane_data.selection_name = "detected_plane";
        plane_data.point_indices = plane_selection;
        plane_data.highlight_color = glm::vec3(0.0f, 1.0f, 0.0f);  // Green
        plane_data.size_multiplier = 1.8f;
        plane_data.show_bounding_box = false;
        plane_data.description = "Detected plane (simulated)";
        
        success = visualization::SelectionVisualizer::CreateHighlight(plane_data, *cloud_ptr_);
        if (success) {
            std::cout << "✓ Created plane highlight with " << plane_selection.size() << " points" << std::endl;
        }
        
        // 3. Simple highlight using convenience method
        auto outlier_selection = SimulateOutlierDetection(50);
        success = visualization::SelectionVisualizer::CreateHighlight(
            outlier_selection,
            glm::vec3(1.0f, 1.0f, 0.0f),  // Yellow
            *cloud_ptr_,
            "outliers",
            1.5f
        );
        if (success) {
            std::cout << "✓ Created outlier highlight with " << outlier_selection.size() << " points" << std::endl;
        }
        
        // 4. Region of interest
        auto roi_selection = SimulateROISelection();
        success = visualization::SelectionVisualizer::CreateHighlight(
            roi_selection,
            glm::vec3(0.0f, 1.0f, 1.0f),  // Cyan
            *cloud_ptr_,
            "roi",
            2.2f
        );
        if (success) {
            std::cout << "✓ Created ROI highlight with " << roi_selection.size() << " points" << std::endl;
        }
        
        std::cout << "\n=== Selection Visualization Complete ===" << std::endl;
        std::cout << "Total highlights created: 4 layers" << std::endl;
        std::cout << "  - Red: Cluster detection result" << std::endl;
        std::cout << "  - Green: Plane detection result" << std::endl;
        std::cout << "  - Yellow: Outlier detection result" << std::endl;
        std::cout << "  - Cyan: Region of interest" << std::endl;
    }
    
private:
    std::vector<size_t> SimulateRandomSelection(size_t count) {
        std::vector<size_t> indices;
        std::mt19937 gen(123);
        std::uniform_int_distribution<size_t> dis(0, cloud_ptr_->GetPointCount() - 1);
        
        for (size_t i = 0; i < count && i < cloud_ptr_->GetPointCount(); ++i) {
            indices.push_back(dis(gen));
        }
        
        // Remove duplicates
        std::sort(indices.begin(), indices.end());
        indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
        
        return indices;
    }
    
    std::vector<size_t> SimulatePlaneDetection() {
        // Simulate plane detection by selecting points near y = -2 (ground plane)
        std::vector<size_t> indices;
        const auto& points = cloud_ptr_->GetPoints();
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (std::abs(points[i].y + 2.0f) < 0.3f) {  // Near ground plane
                indices.push_back(i);
            }
        }
        
        return indices;
    }
    
    std::vector<size_t> SimulateOutlierDetection(size_t count) {
        // Simulate outlier detection by selecting points far from origin
        std::vector<std::pair<float, size_t>> distances;
        const auto& points = cloud_ptr_->GetPoints();
        
        for (size_t i = 0; i < points.size(); ++i) {
            float dist = glm::length(points[i]);
            distances.emplace_back(dist, i);
        }
        
        // Sort by distance and take the furthest points
        std::sort(distances.begin(), distances.end(), std::greater<>());
        
        std::vector<size_t> indices;
        for (size_t i = 0; i < count && i < distances.size(); ++i) {
            indices.push_back(distances[i].second);
        }
        
        return indices;
    }
    
    std::vector<size_t> SimulateROISelection() {
        // Simulate region of interest selection (points in a cube)
        std::vector<size_t> indices;
        const auto& points = cloud_ptr_->GetPoints();
        
        glm::vec3 roi_center(1.5f, 0.0f, 0.5f);
        float roi_size = 1.5f;
        
        for (size_t i = 0; i < points.size(); ++i) {
            glm::vec3 diff = points[i] - roi_center;
            if (std::abs(diff.x) < roi_size && 
                std::abs(diff.y) < roi_size && 
                std::abs(diff.z) < roi_size) {
                indices.push_back(i);
            }
        }
        
        return indices;
    }
    
    void Run() {
        CreateTestScene();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "R: Reset camera to default position" << std::endl;
        std::cout << "ESC: Exit application" << std::endl;
        
        std::cout << "\n=== Visualization Module Architecture Demonstrated ===" << std::endl;
        std::cout << "✓ Clean data contracts (SelectionData)" << std::endl;
        std::cout << "✓ High-level API (SelectionVisualizer::CreateHighlight)" << std::endl;
        std::cout << "✓ Separation of processing and visualization" << std::endl;
        std::cout << "✓ Multi-layer highlighting system" << std::endl;
        std::cout << "✓ External algorithm integration" << std::endl;
        
        // Show viewer
        viewer_.Show();
    }
    
    friend int main(int argc, char* argv[]);
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
    std::unique_ptr<PointCloud> cloud_;
    PointCloud* cloud_ptr_ = nullptr;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Selection Visualizer Integration Demo ===" << std::endl;
    std::cout << "Demonstrating external selection visualization with the new architecture\n" << std::endl;
    
    try {
        SelectionVisualizerDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
/*
 * test_pcl_bridge.cpp
 *
 * Created on: Dec 2024
 * Description: Test PCL bridge utilities for seamless PCL ↔ Renderer integration
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <glm/glm.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "renderer/gl_scene_manager.hpp"
#include "renderer/renderable/grid.hpp"
#include "renderer/renderable/point_cloud.hpp"

#ifdef QUICKVIZ_WITH_PCL
#include "renderer/pcl_bridge/pcl_conversions.hpp"
#include "renderer/pcl_bridge/pcl_visualization.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#endif

using namespace quickviz;

void TestBasicVisualization() {
    std::cout << "=== Testing Basic Point Cloud Visualization ===" << std::endl;
    
    // Create a simple test point cloud
    std::vector<glm::vec4> test_points;
    for (int i = -10; i <= 10; i += 2) {
        for (int j = -10; j <= 10; j += 2) {
            for (int k = -2; k <= 2; k += 2) {
                float intensity = (i + j + k + 30) / 60.0f; // normalized 0-1
                test_points.push_back(glm::vec4(i, j, k, intensity));
            }
        }
    }
    
    std::cout << "Created test point cloud with " << test_points.size() << " points" << std::endl;
    
    // Create viewer and scene
    Viewer viewer;
    auto box = std::make_shared<Box>("main_box");
    box->SetFlexDirection(Styling::FlexDirection::kRow);
    box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    box->SetAlignItems(Styling::AlignItems::kStretch);

    auto gl_sm = std::make_shared<GlSceneManager>("PCL Bridge Test");
    gl_sm->SetAutoLayout(true);
    gl_sm->SetNoTitleBar(true);
    gl_sm->SetFlexGrow(1.0f);
    gl_sm->SetFlexShrink(0.0f);
    
    // Create point cloud
    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPoints(test_points, PointCloud::ColorMode::kScalarField);
    point_cloud->SetScalarRange(0.0f, 1.0f);
    point_cloud->SetPointSize(8.0f);
    point_cloud->SetRenderMode(PointMode::kPoint);
    
    // Add grid for reference
    auto grid = std::make_unique<Grid>(20.0f, 2.0f, glm::vec3(0.7f, 0.7f, 0.7f));
    
    gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
    gl_sm->AddOpenGLObject("grid", std::move(grid));
    
    box->AddChild(gl_sm);
    viewer.AddSceneObject(box);
    
    std::cout << "\n=== Basic Visualization Controls ===" << std::endl;
    std::cout << "Left Mouse: Rotate camera" << std::endl;
    std::cout << "Middle Mouse: Pan/translate in 3D" << std::endl;
    std::cout << "Scroll Wheel: Zoom" << std::endl;
    std::cout << "This demonstrates basic point cloud rendering before PCL integration" << std::endl;
    
    viewer.Show();
}

#ifdef QUICKVIZ_WITH_PCL
void TestPCLBridgeIntegration() {
    std::cout << "\n=== Testing PCL Bridge Integration ===" << std::endl;
    
    // Create a PCL point cloud for testing
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // Generate test data with two distinct clusters
    for (int i = 0; i < 50; ++i) {
        pcl::PointXYZI point;
        // Cluster 1: around origin
        point.x = (rand() % 100) / 100.0f - 0.5f + 2.0f;
        point.y = (rand() % 100) / 100.0f - 0.5f + 2.0f;
        point.z = (rand() % 100) / 100.0f - 0.5f;
        point.intensity = 0.8f;
        pcl_cloud->points.push_back(point);
    }
    
    for (int i = 0; i < 50; ++i) {
        pcl::PointXYZI point;
        // Cluster 2: offset location
        point.x = (rand() % 100) / 100.0f - 0.5f - 2.0f;
        point.y = (rand() % 100) / 100.0f - 0.5f - 2.0f;
        point.z = (rand() % 100) / 100.0f - 0.5f + 1.0f;
        point.intensity = 0.3f;
        pcl_cloud->points.push_back(point);
    }
    
    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
    
    std::cout << "Created PCL point cloud with " << pcl_cloud->points.size() << " points" << std::endl;
    
    // Test clustering visualization
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(pcl_cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_cloud);
    ec.extract(cluster_indices);
    
    std::cout << "Found " << cluster_indices.size() << " clusters" << std::endl;
    
    // Generate cluster statistics
    std::string stats = pcl_bridge::visualization::GenerateClusterStatistics(cluster_indices);
    std::cout << stats << std::endl;
    
    // Test bounding box creation
    auto bounding_box = pcl_bridge::utils::CalculateBoundingBox(*pcl_cloud);
    auto bbox_lines = pcl_bridge::visualization::CreateBoundingBoxLines(
        bounding_box.first, bounding_box.second);
    
    std::cout << "✓ Bounding box calculation successful" << std::endl;
    std::cout << "  Min corner: (" << bounding_box.first.x << ", " 
              << bounding_box.first.y << ", " << bounding_box.first.z << ")" << std::endl;
    std::cout << "  Max corner: (" << bounding_box.second.x << ", " 
              << bounding_box.second.y << ", " << bounding_box.second.z << ")" << std::endl;
    
    // Test centroid calculation
    if (!cluster_indices.empty()) {
        glm::vec3 centroid = pcl_bridge::visualization::CalculateClusterCentroid(
            *pcl_cloud, cluster_indices[0]);
        std::cout << "✓ Cluster centroid calculation successful" << std::endl;
        std::cout << "  First cluster centroid: (" << centroid.x << ", " 
                  << centroid.y << ", " << centroid.z << ")" << std::endl;
    }
    
    // Create visualization first to establish OpenGL context
    Viewer viewer;
    auto box = std::make_shared<Box>("pcl_bridge_box");
    box->SetFlexDirection(Styling::FlexDirection::kRow);
    
    auto gl_sm = std::make_shared<GlSceneManager>("PCL Bridge Integration Test");
    gl_sm->SetAutoLayout(true);
    gl_sm->SetNoTitleBar(true);
    gl_sm->SetFlexGrow(1.0f);
    
    // Now test PCL to Renderer conversion (after OpenGL context is available)
    auto renderer_cloud = std::make_unique<PointCloud>();
    
    try {
        pcl_bridge::ImportFromPCL(*pcl_cloud, 
                                 pcl_bridge::PointConverter<pcl::PointXYZI>(pcl_bridge::converters::PCLXYZIToRenderer),
                                 *renderer_cloud,
                                 PointCloud::ColorMode::kScalarField);
        
        std::cout << "✓ PCL → Renderer conversion successful" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "✗ PCL → Renderer conversion failed: " << e.what() << std::endl;
        return;
    }
    
    // Configure point cloud visualization
    renderer_cloud->SetScalarRange(0.0f, 1.0f);
    renderer_cloud->SetPointSize(10.0f);
    renderer_cloud->SetRenderMode(PointMode::kPoint);
    
    // Add grid for reference
    auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
    
    gl_sm->AddOpenGLObject("pcl_points", std::move(renderer_cloud));
    gl_sm->AddOpenGLObject("grid", std::move(grid));
    
    box->AddChild(gl_sm);
    viewer.AddSceneObject(box);
    
    std::cout << "\n=== PCL Bridge Test Controls ===" << std::endl;
    std::cout << "This visualization shows PCL point cloud converted to renderer format" << std::endl;
    std::cout << "Colors represent intensity values from PCL" << std::endl;
    std::cout << "Two clusters should be visible with different intensity values" << std::endl;
    
    viewer.Show();
}

void TestPCLWorkflow() {
    std::cout << "\n=== Testing Complete PCL Workflow ===" << std::endl;
    
    // This would demonstrate a complete workflow:
    // 1. Load PCD file
    // 2. User interaction for selection
    // 3. Export selection to PCL
    // 4. Run PCL algorithm
    // 5. Visualize results
    
    std::cout << "Complete workflow test placeholder - would require:" << std::endl;
    std::cout << "  1. File selection dialog" << std::endl;
    std::cout << "  2. Interactive selection tools" << std::endl;
    std::cout << "  3. Enhanced PointCloud API for highlighting" << std::endl;
    std::cout << "  4. Overlay rendering for algorithm results" << std::endl;
}
#endif

int main(int argc, char* argv[]) {
    std::cout << "QuickViz PCL Bridge Test Suite" << std::endl;
    std::cout << "==============================" << std::endl;
    
#ifdef QUICKVIZ_WITH_PCL
    std::cout << "✓ PCL support enabled" << std::endl;
#else
    std::cout << "✗ PCL support not available (compile with PCL to enable)" << std::endl;
#endif
    
    if (argc > 1) {
        std::string test_mode = argv[1];
        
        if (test_mode == "basic") {
            TestBasicVisualization();
#ifdef QUICKVIZ_WITH_PCL
        } else if (test_mode == "bridge") {
            TestPCLBridgeIntegration();
        } else if (test_mode == "workflow") {
            TestPCLWorkflow();
#endif
        } else {
            std::cout << "Unknown test mode: " << test_mode << std::endl;
            std::cout << "Available modes: basic";
#ifdef QUICKVIZ_WITH_PCL
            std::cout << ", bridge, workflow";
#endif
            std::cout << std::endl;
            return 1;
        }
    } else {
        // Run basic test by default
        std::cout << "Running basic test (use 'basic', 'bridge', or 'workflow' as argument)" << std::endl;
        TestBasicVisualization();
    }
    
    return 0;
}
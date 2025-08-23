/*
 * @file test_point_cloud.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)  
 * @date 2025-08-23
 * @brief Test for PointCloud rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <memory>
#include <vector>
#include <random>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/point_cloud.hpp"

using namespace quickviz;

void SetupPointCloudScene(GlSceneManager* scene_manager) {
    // Generate random point cloud data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dist(-5.0f, 5.0f);
    std::uniform_real_distribution<float> color_dist(0.0f, 1.0f);
    
    // 1. Basic colored point cloud
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    for (int i = 0; i < 1000; ++i) {
        points.push_back(glm::vec3(pos_dist(gen), pos_dist(gen), pos_dist(gen)));
        colors.push_back(glm::vec3(color_dist(gen), color_dist(gen), color_dist(gen)));
    }
    
    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPoints(points, colors);
    point_cloud->SetPointSize(3.0f);
    scene_manager->AddOpenGLObject("point_cloud", std::move(point_cloud));
    
    // 2. Structured point cloud (sphere pattern)
    std::vector<glm::vec3> sphere_points;
    std::vector<glm::vec3> sphere_colors;
    
    for (int i = 0; i < 500; ++i) {
        float theta = pos_dist(gen) * M_PI;
        float phi = pos_dist(gen) * 2 * M_PI;
        float r = 2.0f + pos_dist(gen) * 0.5f;
        
        float x = r * sin(theta) * cos(phi) + 8.0f;
        float y = r * sin(theta) * sin(phi);
        float z = r * cos(theta);
        
        sphere_points.push_back(glm::vec3(x, y, z));
        sphere_colors.push_back(glm::vec3(1.0f, 0.5f, 0.2f)); // Orange
    }
    
    auto sphere_cloud = std::make_unique<PointCloud>();
    sphere_cloud->SetPoints(sphere_points, sphere_colors);
    sphere_cloud->SetPointSize(2.0f);
    scene_manager->AddOpenGLObject("sphere_cloud", std::move(sphere_cloud));
    
    // 3. Height-based colored plane
    std::vector<glm::vec3> plane_points;
    std::vector<glm::vec3> plane_colors;
    
    for (float x = -3.0f; x <= 3.0f; x += 0.1f) {
        for (float y = -3.0f; y <= 3.0f; y += 0.1f) {
            float z = sin(x) * cos(y) * 0.5f - 8.0f;
            plane_points.push_back(glm::vec3(x, y, z));
            
            // Height-based coloring
            float height_ratio = (z + 8.5f) / 1.0f;  // Normalize to [0,1]
            plane_colors.push_back(glm::vec3(height_ratio, 0.2f, 1.0f - height_ratio));
        }
    }
    
    auto plane_cloud = std::make_unique<PointCloud>();
    plane_cloud->SetPoints(plane_points, plane_colors);
    plane_cloud->SetPointSize(1.5f);
    scene_manager->AddOpenGLObject("plane_cloud", std::move(plane_cloud));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "PointCloud Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing point cloud rendering with various patterns and coloring schemes");
        
        view.AddHelpSection("PointCloud Features Demonstrated", {
            "- Large-scale point cloud rendering (1000+ points)",
            "- Per-point color assignment",
            "- Different point sizes",
            "- Random scattered point distribution",
            "- Structured geometric patterns (sphere)",
            "- Mathematical surface visualization (sine/cosine)",
            "- Height-based color mapping"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Random cloud: 1000 colored points scattered randomly",
            "- Sphere cloud: 500 orange points in spherical pattern at (8,0,0)",
            "- Plane cloud: Mathematical surface at z=-8 with height coloring",
            "- Variable point sizes: 1.5f to 3.0f pixels",
            "- Grid and coordinate frame for spatial reference"
        });
        
        view.AddHelpSection("Technical Details", {
            "- OpenGL point primitives (GL_POINTS)",
            "- Vertex buffer objects for efficient rendering",
            "- Per-vertex color attributes",
            "- Point size control via OpenGL",
            "- 3D depth testing for proper occlusion",
            "- Large dataset handling (1500+ points total)"
        });
        
        view.AddHelpSection("Applications", {
            "- Lidar scan visualization",
            "- 3D sensor data display",
            "- Scientific data plotting",
            "- Geographic point data (GPS coordinates)",
            "- Particle system rendering",
            "- Statistical data visualization in 3D"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupPointCloudScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
/*
 * @file test_frustum.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for Frustum rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/frustum.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

void SetupFrustumScene(GlSceneManager* scene_manager) {
    // Add grid for reference
    auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // Camera frustum - front center
    auto camera_frustum = std::make_unique<Frustum>();
    camera_frustum->SetFromPerspective(
        glm::vec3(0.0f, 0.0f, 0.0f),        // Origin at center
        glm::vec3(0.0f, 0.0f, -1.0f),       // Looking down -Z
        45.0f,                               // 45° FOV
        16.0f / 9.0f,                        // 16:9 aspect ratio
        0.5f,                                // Near distance
        4.0f                                 // Far distance
    );
    camera_frustum->SetColor(glm::vec3(0.2f, 0.8f, 0.2f));          // Green
    camera_frustum->SetRenderMode(Frustum::RenderMode::kTransparent);
    camera_frustum->SetTransparency(0.3f);
    camera_frustum->SetShowSideFaces(true);
    camera_frustum->SetShowNearFace(false);  // Don't show near face to see inside
    camera_frustum->SetShowFarFace(true);
    scene_manager->AddOpenGLObject("camera_frustum", std::move(camera_frustum));
    
    // LiDAR sector - left side
    auto lidar_frustum = std::make_unique<Frustum>();
    lidar_frustum->SetFromLidarSector(
        glm::vec3(-3.0f, 0.0f, 0.0f),        // Left position
        glm::vec3(1.0f, 0.0f, 0.0f),         // Looking right (+X)
        90.0f,                               // 90° horizontal FOV
        30.0f,                               // 30° vertical FOV
        0.2f,                                // Min range
        6.0f                                 // Max range
    );
    lidar_frustum->SetColor(glm::vec3(1.0f, 0.4f, 0.2f));          // Orange
    lidar_frustum->SetRenderMode(Frustum::RenderMode::kWireframe);
    lidar_frustum->SetWireframeColor(glm::vec3(1.0f, 0.6f, 0.0f));
    lidar_frustum->SetWireframeWidth(2.0f);
    scene_manager->AddOpenGLObject("lidar_frustum", std::move(lidar_frustum));
    
    // Radar cone - right side
    auto radar_frustum = std::make_unique<Frustum>();
    radar_frustum->SetFromPerspective(
        glm::vec3(3.0f, 0.0f, 0.0f),         // Right position
        glm::vec3(-1.0f, 0.0f, 0.0f),        // Looking left (-X)
        60.0f,                               // 60° FOV
        1.0f,                                // Square aspect
        1.0f,                                // Near distance
        8.0f                                 // Far distance  
    );
    radar_frustum->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));          // Magenta
    radar_frustum->SetRenderMode(Frustum::RenderMode::kTransparent);
    radar_frustum->SetTransparency(0.4f);
    radar_frustum->SetShowCenterLine(true);
    radar_frustum->SetCenterLineColor(glm::vec3(1.0f, 0.0f, 1.0f));
    scene_manager->AddOpenGLObject("radar_frustum", std::move(radar_frustum));
    
    // Solid frustum - elevated
    auto solid_frustum = std::make_unique<Frustum>();
    solid_frustum->SetFromPerspective(
        glm::vec3(0.0f, 3.0f, 0.0f),         // Elevated position
        glm::vec3(0.0f, -1.0f, -0.5f),       // Looking down and forward
        75.0f,                               // Wide FOV
        1.5f,                                // 3:2 aspect
        0.8f,                                // Near distance
        3.5f                                 // Far distance
    );
    solid_frustum->SetColor(glm::vec3(0.2f, 0.6f, 1.0f));          // Blue
    solid_frustum->SetRenderMode(Frustum::RenderMode::kSolid);
    solid_frustum->SetShowCornerMarkers(true);
    solid_frustum->SetCornerMarkerSize(0.1f);
    scene_manager->AddOpenGLObject("solid_frustum", std::move(solid_frustum));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Frustum Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing frustum rendering for sensor field-of-view visualization");
        
        view.AddHelpSection("Frustum Features Demonstrated", {
            "- Different field-of-view angles (45° to 120°)",
            "- Various near/far distances for sensor ranges",
            "- Different aspect ratios for sensor types",
            "- Positioning and orientation in 3D space",
            "- Transparency for overlapping FOV visualization",
            "- Wireframe mode for clear boundary display"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Green transparent camera frustum: 45° FOV, 16:9 aspect ratio at center",
            "- Orange wireframe LiDAR sector: 90° horizontal, 30° vertical on left",  
            "- Magenta transparent radar cone: 60° FOV with center line on right",
            "- Blue solid frustum: 75° FOV elevated position with corner markers",
            "- Grid reference plane for spatial orientation"
        });
        
        view.AddHelpSection("Robotics Applications", {
            "- Camera field-of-view visualization",
            "- Lidar scanning cone display",
            "- Radar detection zone boundaries",
            "- Ultrasonic sensor range visualization", 
            "- Robot perception workspace display",
            "- Multi-sensor coverage analysis"
        });
        
        view.AddHelpSection("Parameters", {
            "- FOV angle: Horizontal field-of-view in degrees",
            "- Aspect ratio: Width/height ratio of sensor",
            "- Near distance: Minimum detection range",
            "- Far distance: Maximum detection range",
            "- Position: 3D location of sensor origin",
            "- Orientation: Pitch, yaw, roll rotation angles"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupFrustumScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
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

using namespace quickviz;

void SetupFrustumScene(GlSceneManager* scene_manager) {
    // 1. Camera frustum - Red
    auto camera_frustum = std::make_unique<Frustum>();
    camera_frustum->SetFromPerspective(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), 60.0f, 1.0f, 0.1f, 5.0f);
    camera_frustum->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("camera", std::move(camera_frustum));
    
    // 2. Lidar frustum - Green
    auto lidar_frustum = std::make_unique<Frustum>();
    lidar_frustum->SetFromLidarSector(glm::vec3(-3.0f, 0.0f, 0.0f), glm::vec3(0.707f, 0.0f, 0.707f), 90.0f, 30.0f, 0.2f, 10.0f);
    lidar_frustum->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("lidar", std::move(lidar_frustum));
    
    // 3. Radar frustum - Blue  
    auto radar_frustum = std::make_unique<Frustum>();
    radar_frustum->SetFromPerspective(glm::vec3(3.0f, 0.0f, 0.0f), glm::vec3(-0.707f, 0.0f, 0.707f), 120.0f, 0.5f, 0.5f, 15.0f);
    radar_frustum->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    scene_manager->AddOpenGLObject("radar", std::move(radar_frustum));
    
    // 4. Transparent sensor FOV - Yellow
    auto sensor_frustum = std::make_unique<Frustum>();
    sensor_frustum->SetFromPerspective(glm::vec3(0.0f, 3.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), 45.0f, 1.5f, 0.1f, 8.0f);
    sensor_frustum->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    sensor_frustum->SetRenderMode(Frustum::RenderMode::kTransparent);
    sensor_frustum->SetTransparency(0.4f);
    scene_manager->AddOpenGLObject("sensor", std::move(sensor_frustum));
    
    // 5. Wireframe frustum - Purple
    auto wireframe_frustum = std::make_unique<Frustum>();
    wireframe_frustum->SetFromPerspective(glm::vec3(0.0f, -3.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), 75.0f, 1.0f, 0.3f, 6.0f);
    wireframe_frustum->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
    wireframe_frustum->SetRenderMode(Frustum::RenderMode::kWireframe);
    scene_manager->AddOpenGLObject("wireframe", std::move(wireframe_frustum));
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
            "- Red frustum: Camera FOV (60°, 0.1-5.0m range)",
            "- Green frustum: Lidar FOV (90°, 0.2-10.0m, rotated 45°)",
            "- Blue frustum: Radar FOV (120°, 0.5-15.0m, rotated -45°)",
            "- Yellow frustum: Sensor FOV (45°, transparent, at (0,3,0))",
            "- Purple frustum: Wireframe mode (75°, at (0,-3,0))"
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
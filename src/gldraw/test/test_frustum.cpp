/*
 * @file test_frustum.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Visual test for Frustum renderable - demonstrates sensor FOV visualization for robotics
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/frustum.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/arrow.hpp"
#include "gldraw/renderable/sphere.hpp"

using namespace quickviz;

class FrustumDemo {
public:
    FrustumDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager
        scene_manager_ = std::make_shared<GlSceneManager>("Frustum Demo");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateDemoScene() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // Demo 1: Camera FOV (perspective frustum)
        CreateCameraFOV();
        
        // Demo 2: LiDAR sensor coverage (sector frustum)
        CreateLidarCoverage();
        
        // Demo 3: Radar detection zone (orthographic frustum)
        CreateRadarZone();
        
        // Demo 4: Spotlight illumination (perspective with different settings)
        CreateSpotlight();
        
        // Demo 5: Security camera coverage (multiple frustums)
        CreateSecurityCameras();
        
        std::cout << "\n=== Frustum Rendering Demo ===\n";
        std::cout << "Demonstrating sensor field of view visualization for robotics:\n";
        std::cout << "• Blue translucent: Camera field of view (perspective)\n";
        std::cout << "• Green wireframe: LiDAR sector coverage\n";
        std::cout << "• Red transparent: Radar detection zone (orthographic)\n";
        std::cout << "• Yellow outline: Spotlight illumination volume\n";
        std::cout << "• Cyan points: Security camera positions with FOV\n\n";
    }
    
    void CreateCameraFOV() {
        // RGB-D camera at origin looking forward
        glm::vec3 camera_pos(-3.0f, 1.5f, 0.0f);
        glm::vec3 camera_dir(1.0f, -0.2f, 0.0f);
        
        auto camera_frustum = std::make_unique<Frustum>();
        camera_frustum->SetFromPerspective(camera_pos, camera_dir, 60.0f, 16.0f/9.0f, 0.1f, 5.0f);
        camera_frustum->SetColor(glm::vec3(0.2f, 0.4f, 0.8f));
        camera_frustum->SetTransparency(0.3f);
        camera_frustum->SetRenderMode(Frustum::RenderMode::kTransparent);
        camera_frustum->SetShowNearFace(false);
        camera_frustum->SetShowFarFace(true);
        camera_frustum->SetShowSideFaces(true);
        camera_frustum->SetShowCenterLine(true);
        camera_frustum->SetCenterLineColor(glm::vec3(0.0f, 0.8f, 1.0f));
        scene_manager_->AddOpenGLObject("camera_fov", std::move(camera_frustum));
        
        // Add camera position marker
        auto camera_marker = std::make_unique<Sphere>();
        camera_marker->SetCenter(camera_pos);
        camera_marker->SetRadius(0.1f);
        camera_marker->SetColor(glm::vec3(0.2f, 0.4f, 0.8f));
        camera_marker->SetRenderMode(Sphere::RenderMode::kSolid);
        scene_manager_->AddOpenGLObject("camera_marker", std::move(camera_marker));
    }
    
    void CreateLidarCoverage() {
        // 2D LiDAR sensor coverage area
        glm::vec3 lidar_pos(0.0f, 1.0f, -3.0f);
        glm::vec3 lidar_dir(0.0f, -0.1f, 1.0f);
        
        auto lidar_frustum = std::make_unique<Frustum>();
        lidar_frustum->SetFromLidarSector(lidar_pos, lidar_dir, 120.0f, 15.0f, 0.2f, 8.0f);
        lidar_frustum->SetColor(glm::vec3(0.0f, 0.8f, 0.0f));
        lidar_frustum->SetRenderMode(Frustum::RenderMode::kWireframe);
        lidar_frustum->SetWireframeColor(glm::vec3(0.0f, 1.0f, 0.0f));
        lidar_frustum->SetWireframeWidth(2.0f);
        lidar_frustum->SetShowNearFace(true);
        lidar_frustum->SetShowFarFace(true);
        lidar_frustum->SetShowSideFaces(true);
        scene_manager_->AddOpenGLObject("lidar_coverage", std::move(lidar_frustum));
        
        // Add LiDAR sensor marker with direction arrow
        auto lidar_marker = std::make_unique<Sphere>();
        lidar_marker->SetCenter(lidar_pos);
        lidar_marker->SetRadius(0.08f);
        lidar_marker->SetColor(glm::vec3(0.0f, 0.8f, 0.0f));
        lidar_marker->SetRenderMode(Sphere::RenderMode::kSolid);
        scene_manager_->AddOpenGLObject("lidar_marker", std::move(lidar_marker));
        
        auto lidar_arrow = std::make_unique<Arrow>();
        lidar_arrow->SetDirection(lidar_pos, lidar_dir, 1.0f);
        lidar_arrow->SetColor(glm::vec3(0.0f, 0.6f, 0.0f));
        lidar_arrow->SetShaftRadius(0.05f);
        lidar_arrow->SetHeadRadius(0.1f);
        scene_manager_->AddOpenGLObject("lidar_arrow", std::move(lidar_arrow));
    }
    
    void CreateRadarZone() {
        // Automotive radar detection zone (orthographic)
        glm::vec3 radar_pos(3.0f, 0.5f, 0.0f);
        glm::vec3 radar_dir(-1.0f, 0.0f, 0.0f);
        
        auto radar_frustum = std::make_unique<Frustum>();
        radar_frustum->SetFromOrthographic(radar_pos, radar_dir, 4.0f, 3.0f, 1.0f, 15.0f);
        radar_frustum->SetColor(glm::vec3(0.8f, 0.2f, 0.2f));
        radar_frustum->SetTransparency(0.4f);
        radar_frustum->SetRenderMode(Frustum::RenderMode::kTransparent);
        radar_frustum->SetShowNearFace(true);
        radar_frustum->SetShowFarFace(true);
        radar_frustum->SetShowSideFaces(true);
        radar_frustum->SetShowCenterLine(true);
        radar_frustum->SetCenterLineColor(glm::vec3(1.0f, 0.0f, 0.0f));
        scene_manager_->AddOpenGLObject("radar_zone", std::move(radar_frustum));
        
        // Add radar sensor marker
        auto radar_marker = std::make_unique<Sphere>();
        radar_marker->SetCenter(radar_pos);
        radar_marker->SetRadius(0.12f);
        radar_marker->SetColor(glm::vec3(0.8f, 0.2f, 0.2f));
        radar_marker->SetRenderMode(Sphere::RenderMode::kSolid);
        scene_manager_->AddOpenGLObject("radar_marker", std::move(radar_marker));
    }
    
    void CreateSpotlight() {
        // Security spotlight with narrow cone
        glm::vec3 light_pos(0.0f, 4.0f, 0.0f);
        glm::vec3 light_dir(0.2f, -1.0f, 0.3f);
        
        auto spotlight_frustum = std::make_unique<Frustum>();
        spotlight_frustum->SetFromPerspective(light_pos, light_dir, 30.0f, 1.0f, 0.5f, 6.0f);
        spotlight_frustum->SetColor(glm::vec3(1.0f, 1.0f, 0.2f));
        spotlight_frustum->SetRenderMode(Frustum::RenderMode::kOutline);
        spotlight_frustum->SetWireframeColor(glm::vec3(1.0f, 1.0f, 0.0f));
        spotlight_frustum->SetWireframeWidth(3.0f);
        spotlight_frustum->SetShowNearFace(false);
        spotlight_frustum->SetShowFarFace(true);
        spotlight_frustum->SetShowSideFaces(true);
        scene_manager_->AddOpenGLObject("spotlight", std::move(spotlight_frustum));
        
        // Add light source marker
        auto light_marker = std::make_unique<Sphere>();
        light_marker->SetCenter(light_pos);
        light_marker->SetRadius(0.15f);
        light_marker->SetColor(glm::vec3(1.0f, 1.0f, 0.2f));
        light_marker->SetRenderMode(Sphere::RenderMode::kSolid);
        scene_manager_->AddOpenGLObject("light_marker", std::move(light_marker));
    }
    
    void CreateSecurityCameras() {
        // Multiple security cameras with overlapping coverage
        std::vector<glm::vec3> camera_positions = {
            glm::vec3(-4.0f, 3.0f, -4.0f),
            glm::vec3(4.0f, 3.0f, -4.0f),
            glm::vec3(-4.0f, 3.0f, 4.0f),
            glm::vec3(4.0f, 3.0f, 4.0f)
        };
        
        std::vector<glm::vec3> camera_directions = {
            glm::vec3(1.0f, -0.5f, 1.0f),
            glm::vec3(-1.0f, -0.5f, 1.0f),
            glm::vec3(1.0f, -0.5f, -1.0f),
            glm::vec3(-1.0f, -0.5f, -1.0f)
        };
        
        for (size_t i = 0; i < camera_positions.size(); ++i) {
            // Create camera frustum
            auto cam_frustum = std::make_unique<Frustum>();
            cam_frustum->SetFromPerspective(camera_positions[i], camera_directions[i], 
                                          45.0f, 4.0f/3.0f, 0.3f, 8.0f);
            cam_frustum->SetColor(glm::vec3(0.0f, 0.8f, 0.8f));
            cam_frustum->SetRenderMode(Frustum::RenderMode::kPoints);
            cam_frustum->SetWireframeColor(glm::vec3(0.0f, 1.0f, 1.0f));
            cam_frustum->SetShowCornerMarkers(true);
            cam_frustum->SetCornerMarkerSize(0.05f);
            cam_frustum->SetShowCenterLine(true);
            cam_frustum->SetCenterLineColor(glm::vec3(0.0f, 0.6f, 0.6f));
            scene_manager_->AddOpenGLObject("security_cam_" + std::to_string(i), std::move(cam_frustum));
            
            // Add camera housing marker
            auto cam_marker = std::make_unique<Sphere>();
            cam_marker->SetCenter(camera_positions[i]);
            cam_marker->SetRadius(0.1f);
            cam_marker->SetColor(glm::vec3(0.0f, 0.8f, 0.8f));
            cam_marker->SetRenderMode(Sphere::RenderMode::kSolid);
            scene_manager_->AddOpenGLObject("security_marker_" + std::to_string(i), std::move(cam_marker));
        }
    }
    
    void Run() {
        CreateDemoScene();
        
        std::cout << "=== Camera Controls ===\n";
        std::cout << "Left Mouse: Rotate camera (orbit mode)\n";
        std::cout << "Middle Mouse: Translate/Pan in 3D space\n";
        std::cout << "Scroll Wheel: Zoom in/out\n";
        std::cout << "R: Reset camera to default position\n";
        std::cout << "ESC: Exit application\n\n";
        
        std::cout << "=== Frustum Features Demonstrated ===\n";
        std::cout << "✓ Perspective projection (camera, spotlight)\n";
        std::cout << "✓ Orthographic projection (radar)\n";
        std::cout << "✓ LiDAR sector configuration\n";
        std::cout << "✓ Multiple render modes (solid, wireframe, transparent, outline, points)\n";
        std::cout << "✓ Face visibility control (near, far, sides)\n";
        std::cout << "✓ Center lines and corner markers\n";
        std::cout << "✓ Color and transparency customization\n";
        std::cout << "✓ Integration with sensor position markers\n\n";
        
        // Show viewer
        viewer_.Show();
    }
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Frustum Renderable Demo ===\n";
    std::cout << "Demonstrating sensor field of view visualization for robotics\n\n";
    
    try {
        FrustumDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
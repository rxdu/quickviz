/*
 * @file test_sphere.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Manual test for sphere rendering functionality
 *
 * This test creates a window displaying different sphere types for waypoints and ranges.
 * Run this test to visually verify sphere functionality for robotics applications.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

class SphereTestDemo {
public:
    SphereTestDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager with proper layout settings
        scene_manager_ = std::make_shared<GlSceneManager>("Sphere Rendering Test");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateTestSpheres() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // Add coordinate frame at origin
        auto frame = std::make_unique<CoordinateFrame>(1.5f);
        scene_manager_->AddOpenGLObject("frame", std::move(frame));
        
        // 1. Simple solid sphere at origin - Red
        auto solid_sphere = std::make_unique<Sphere>(
            glm::vec3(0.0f, 0.0f, 0.0f), 1.0f
        );
        solid_sphere->SetRenderMode(Sphere::RenderMode::kSolid);
        solid_sphere->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        scene_manager_->AddOpenGLObject("solid", std::move(solid_sphere));
        
        // 2. Wireframe sphere - Green
        auto wireframe_sphere = std::make_unique<Sphere>();
        wireframe_sphere->SetCenter(glm::vec3(3.0f, 0.0f, 0.0f));
        wireframe_sphere->SetRadius(0.8f);
        wireframe_sphere->SetRenderMode(Sphere::RenderMode::kWireframe);
        wireframe_sphere->SetWireframeColor(glm::vec3(0.0f, 1.0f, 0.0f));
        wireframe_sphere->SetWireframeWidth(2.0f);
        scene_manager_->AddOpenGLObject("wireframe", std::move(wireframe_sphere));
        
        // 3. Transparent sphere - Blue
        auto transparent_sphere = std::make_unique<Sphere>(
            glm::vec3(-3.0f, 0.0f, 0.0f), 1.2f
        );
        transparent_sphere->SetRenderMode(Sphere::RenderMode::kTransparent);
        transparent_sphere->SetColor(glm::vec3(0.2f, 0.2f, 0.9f));
        transparent_sphere->SetOpacity(0.4f);
        scene_manager_->AddOpenGLObject("transparent", std::move(transparent_sphere));
        
        // 4. Detection range sphere - Yellow, large
        auto range_sphere = std::make_unique<Sphere>(
            glm::vec3(0.0f, 0.0f, 4.0f), 2.0f
        );
        range_sphere->SetRenderMode(Sphere::RenderMode::kTransparent);
        range_sphere->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        range_sphere->SetOpacity(0.2f);
        scene_manager_->AddOpenGLObject("detection_range", std::move(range_sphere));
        
        // 5. Waypoint marker with poles - Cyan
        auto waypoint_sphere = std::make_unique<Sphere>(
            glm::vec3(2.0f, 2.0f, 2.0f), 0.5f
        );
        waypoint_sphere->SetRenderMode(Sphere::RenderMode::kWireframe);
        waypoint_sphere->SetWireframeColor(glm::vec3(0.0f, 1.0f, 1.0f));
        waypoint_sphere->SetShowPoles(true, 8.0f);
        scene_manager_->AddOpenGLObject("waypoint", std::move(waypoint_sphere));
        
        // 6. Sphere with equator - Purple
        auto equator_sphere = std::make_unique<Sphere>(
            glm::vec3(-2.0f, 2.0f, -2.0f), 0.8f
        );
        equator_sphere->SetRenderMode(Sphere::RenderMode::kTransparent);
        equator_sphere->SetColor(glm::vec3(0.7f, 0.2f, 0.7f));
        equator_sphere->SetOpacity(0.6f);
        equator_sphere->SetShowEquator(true, glm::vec3(1.0f, 1.0f, 0.0f));
        scene_manager_->AddOpenGLObject("equator_sphere", std::move(equator_sphere));
        
        // 7. Point cloud sphere - Orange
        auto point_sphere = std::make_unique<Sphere>(
            glm::vec3(4.0f, 1.0f, -2.0f), 0.6f
        );
        point_sphere->SetRenderMode(Sphere::RenderMode::kPoints);
        point_sphere->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));
        point_sphere->SetResolution(15, 20);  // Medium resolution for points
        scene_manager_->AddOpenGLObject("point_sphere", std::move(point_sphere));
        
        // 8. High-resolution smooth sphere - White
        auto hires_sphere = std::make_unique<Sphere>(
            glm::vec3(0.0f, 3.0f, 0.0f), 0.7f
        );
        hires_sphere->SetRenderMode(Sphere::RenderMode::kSolid);
        hires_sphere->SetColor(glm::vec3(0.9f, 0.9f, 0.9f));
        hires_sphere->SetResolution(40, 40);  // High resolution for smooth appearance
        scene_manager_->AddOpenGLObject("hires", std::move(hires_sphere));
        
        // 9. Multiple small spheres (obstacle field)
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 3; ++j) {
                float x = -4.0f + i * 2.0f;
                float z = -8.0f - j * 1.5f;
                
                auto obstacle_sphere = std::make_unique<Sphere>(
                    glm::vec3(x, 0.3f, z), 0.3f
                );
                obstacle_sphere->SetRenderMode(Sphere::RenderMode::kSolid);
                obstacle_sphere->SetColor(glm::vec3(0.6f, 0.3f, 0.1f));  // Brown
                obstacle_sphere->SetResolution(12, 12);  // Lower resolution for many spheres
                
                scene_manager_->AddOpenGLObject(
                    "obstacle_" + std::to_string(i) + "_" + std::to_string(j),
                    std::move(obstacle_sphere)
                );
            }
        }
        
        // 10. Animated sphere with transform (simple rotation)
        auto rotating_sphere = std::make_unique<Sphere>(
            glm::vec3(-5.0f, 1.0f, 0.0f), 0.6f
        );
        rotating_sphere->SetRenderMode(Sphere::RenderMode::kTransparent);
        rotating_sphere->SetColor(glm::vec3(0.8f, 0.4f, 0.9f));  // Pink
        rotating_sphere->SetOpacity(0.7f);
        rotating_sphere->SetShowEquator(true, glm::vec3(1.0f, 0.0f, 0.0f));  // Red equator
        // Note: In a real application, you'd update the transform in a render loop
        scene_manager_->AddOpenGLObject("rotating", std::move(rotating_sphere));
        
        std::cout << "\nCreated test scene with:" << std::endl;
        std::cout << "  - Reference grid and coordinate frame" << std::endl;
        std::cout << "  - Red solid sphere (basic)" << std::endl;
        std::cout << "  - Green wireframe sphere" << std::endl;
        std::cout << "  - Blue transparent sphere" << std::endl;
        std::cout << "  - Yellow detection range (large, transparent)" << std::endl;
        std::cout << "  - Cyan waypoint with pole markers" << std::endl;
        std::cout << "  - Purple sphere with yellow equator" << std::endl;
        std::cout << "  - Orange point cloud sphere" << std::endl;
        std::cout << "  - White high-resolution smooth sphere" << std::endl;
        std::cout << "  - Brown obstacle field (5x3 grid)" << std::endl;
        std::cout << "  - Pink sphere with red equator" << std::endl;
    }
    
    void Run() {
        CreateTestSpheres();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "R: Reset camera to default position" << std::endl;
        std::cout << "ESC: Exit application" << std::endl;
        
        std::cout << "\n=== Sphere Features Demonstrated ===" << std::endl;
        std::cout << "- Solid, wireframe, transparent, and point rendering modes" << std::endl;
        std::cout << "- Variable sizes and positions" << std::endl;
        std::cout << "- Pole and equator highlighting" << std::endl;
        std::cout << "- Different resolutions for quality/performance trade-off" << std::endl;
        std::cout << "- Transparency with proper alpha blending" << std::endl;
        std::cout << "- Multiple use cases: waypoints, ranges, obstacles" << std::endl;
        
        // Show viewer
        viewer_.Show();
    }
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Sphere Rendering Test ===" << std::endl;
    std::cout << "Testing sphere rendering for waypoints, ranges, and detection zones\n" << std::endl;
    
    try {
        SphereTestDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
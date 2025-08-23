/*
 * @file test_cylinder.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Manual test for cylinder rendering functionality
 *
 * This test creates a window displaying different cylinder types for obstacles and structures.
 * Run this test to visually verify cylinder functionality for robotics applications.
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
#include "gldraw/renderable/cylinder.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

class CylinderTestDemo {
public:
    CylinderTestDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager with proper layout settings
        scene_manager_ = std::make_shared<GlSceneManager>("Cylinder Rendering Test");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateTestCylinders() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // Add coordinate frame at origin
        auto frame = std::make_unique<CoordinateFrame>(1.5f);
        scene_manager_->AddOpenGLObject("frame", std::move(frame));
        
        // 1. Basic vertical cylinder at origin - Red
        auto vertical_cylinder = std::make_unique<Cylinder>(
            glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 0.5f
        );
        vertical_cylinder->SetRenderMode(Cylinder::RenderMode::kSolid);
        vertical_cylinder->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        scene_manager_->AddOpenGLObject("vertical", std::move(vertical_cylinder));
        
        // 2. Horizontal cylinder - Green
        auto horizontal_cylinder = std::make_unique<Cylinder>(
            glm::vec3(-1.0f, 0.5f, 3.0f),
            glm::vec3(1.0f, 0.5f, 3.0f),
            0.3f
        );
        horizontal_cylinder->SetRenderMode(Cylinder::RenderMode::kSolid);
        horizontal_cylinder->SetColor(glm::vec3(0.0f, 0.8f, 0.0f));
        scene_manager_->AddOpenGLObject("horizontal", std::move(horizontal_cylinder));
        
        // 3. Wireframe cylinder - Blue
        auto wireframe_cylinder = std::make_unique<Cylinder>(
            glm::vec3(3.0f, 0.0f, 0.0f), 1.5f, 0.4f
        );
        wireframe_cylinder->SetRenderMode(Cylinder::RenderMode::kWireframe);
        wireframe_cylinder->SetWireframeColor(glm::vec3(0.0f, 0.0f, 1.0f));
        wireframe_cylinder->SetWireframeWidth(2.0f);
        scene_manager_->AddOpenGLObject("wireframe", std::move(wireframe_cylinder));
        
        // 4. Transparent cylinder - Yellow
        auto transparent_cylinder = std::make_unique<Cylinder>(
            glm::vec3(-3.0f, 0.0f, 0.0f), 1.8f, 0.6f
        );
        transparent_cylinder->SetRenderMode(Cylinder::RenderMode::kTransparent);
        transparent_cylinder->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        transparent_cylinder->SetOpacity(0.4f);
        scene_manager_->AddOpenGLObject("transparent", std::move(transparent_cylinder));
        
        // 5. Diagonal cylinder (column) - Purple
        auto diagonal_cylinder = std::make_unique<Cylinder>(
            glm::vec3(2.0f, 0.0f, 2.0f),
            glm::vec3(3.0f, 2.0f, 3.0f),
            0.2f
        );
        diagonal_cylinder->SetRenderMode(Cylinder::RenderMode::kSolid);
        diagonal_cylinder->SetColor(glm::vec3(0.7f, 0.2f, 0.7f));
        scene_manager_->AddOpenGLObject("diagonal", std::move(diagonal_cylinder));
        
        // 6. Cylinder without caps - Cyan
        auto no_caps_cylinder = std::make_unique<Cylinder>(
            glm::vec3(-2.0f, 0.0f, -2.0f), 1.0f, 0.3f
        );
        no_caps_cylinder->SetRenderMode(Cylinder::RenderMode::kSolid);
        no_caps_cylinder->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
        no_caps_cylinder->SetShowCaps(false);
        scene_manager_->AddOpenGLObject("no_caps", std::move(no_caps_cylinder));
        
        // 7. Outline only cylinder - Orange
        auto outline_cylinder = std::make_unique<Cylinder>(
            glm::vec3(4.0f, 1.0f, -2.0f), 1.2f, 0.4f
        );
        outline_cylinder->SetRenderMode(Cylinder::RenderMode::kOutline);
        outline_cylinder->SetWireframeColor(glm::vec3(1.0f, 0.5f, 0.0f));
        outline_cylinder->SetWireframeWidth(3.0f);
        scene_manager_->AddOpenGLObject("outline", std::move(outline_cylinder));
        
        // 8. High-resolution smooth cylinder - White
        auto hires_cylinder = std::make_unique<Cylinder>(
            glm::vec3(0.0f, 3.0f, 0.0f), 1.0f, 0.5f
        );
        hires_cylinder->SetRenderMode(Cylinder::RenderMode::kSolid);
        hires_cylinder->SetColor(glm::vec3(0.9f, 0.9f, 0.9f));
        hires_cylinder->SetResolution(40);  // High resolution
        scene_manager_->AddOpenGLObject("hires", std::move(hires_cylinder));
        
        // 9. Low-resolution cylinder - Brown
        auto lowres_cylinder = std::make_unique<Cylinder>(
            glm::vec3(-4.0f, 1.0f, -1.0f), 1.5f, 0.4f
        );
        lowres_cylinder->SetRenderMode(Cylinder::RenderMode::kSolid);
        lowres_cylinder->SetColor(glm::vec3(0.6f, 0.3f, 0.1f));
        lowres_cylinder->SetResolution(6);  // Low resolution (hexagonal)
        scene_manager_->AddOpenGLObject("lowres", std::move(lowres_cylinder));
        
        // 10. Forest of obstacle cylinders
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 3; ++j) {
                float x = -8.0f + i * 1.5f;
                float z = -6.0f - j * 1.0f;
                float height = 0.8f + (i + j) * 0.2f;
                
                auto obstacle_cylinder = std::make_unique<Cylinder>(
                    glm::vec3(x, 0.0f, z), height, 0.15f
                );
                obstacle_cylinder->SetRenderMode(Cylinder::RenderMode::kSolid);
                obstacle_cylinder->SetColor(glm::vec3(0.4f, 0.2f, 0.0f));  // Dark brown
                obstacle_cylinder->SetResolution(12);  // Medium resolution
                
                scene_manager_->AddOpenGLObject(
                    "obstacle_" + std::to_string(i) + "_" + std::to_string(j),
                    std::move(obstacle_cylinder)
                );
            }
        }
        
        // 11. Pipe/tube system (multiple connected cylinders)
        std::vector<glm::vec3> pipe_points = {
            glm::vec3(5.0f, 0.2f, -5.0f),
            glm::vec3(6.0f, 0.2f, -5.0f),
            glm::vec3(6.0f, 1.2f, -5.0f),
            glm::vec3(6.0f, 1.2f, -3.0f),
            glm::vec3(4.0f, 1.2f, -3.0f)
        };
        
        for (size_t i = 0; i < pipe_points.size() - 1; ++i) {
            auto pipe_segment = std::make_unique<Cylinder>(
                pipe_points[i], pipe_points[i + 1], 0.1f
            );
            pipe_segment->SetRenderMode(Cylinder::RenderMode::kSolid);
            pipe_segment->SetColor(glm::vec3(0.5f, 0.5f, 0.5f));  // Gray
            pipe_segment->SetResolution(16);
            
            scene_manager_->AddOpenGLObject(
                "pipe_" + std::to_string(i),
                std::move(pipe_segment)
            );
        }
        
        std::cout << "\nCreated test scene with:" << std::endl;
        std::cout << "  - Reference grid and coordinate frame" << std::endl;
        std::cout << "  - Red vertical cylinder (basic)" << std::endl;
        std::cout << "  - Green horizontal cylinder" << std::endl;
        std::cout << "  - Blue wireframe cylinder" << std::endl;
        std::cout << "  - Yellow transparent cylinder" << std::endl;
        std::cout << "  - Purple diagonal cylinder" << std::endl;
        std::cout << "  - Cyan cylinder without caps" << std::endl;
        std::cout << "  - Orange outline-only cylinder" << std::endl;
        std::cout << "  - White high-resolution cylinder" << std::endl;
        std::cout << "  - Brown low-resolution (hexagonal) cylinder" << std::endl;
        std::cout << "  - Brown obstacle forest (5x3 grid)" << std::endl;
        std::cout << "  - Gray pipe system (connected segments)" << std::endl;
    }
    
    void Run() {
        CreateTestCylinders();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "R: Reset camera to default position" << std::endl;
        std::cout << "ESC: Exit application" << std::endl;
        
        std::cout << "\n=== Cylinder Features Demonstrated ===" << std::endl;
        std::cout << "- Vertical, horizontal, and diagonal orientations" << std::endl;
        std::cout << "- Solid, wireframe, transparent, and outline modes" << std::endl;
        std::cout << "- Variable radius and height" << std::endl;
        std::cout << "- Cap visibility control" << std::endl;
        std::cout << "- Different resolutions for quality/performance trade-off" << std::endl;
        std::cout << "- Multiple use cases: obstacles, columns, pipes" << std::endl;
        
        // Show viewer
        viewer_.Show();
    }
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Cylinder Rendering Test ===" << std::endl;
    std::cout << "Testing cylinder rendering for obstacles, structures, and pipes\n" << std::endl;
    
    try {
        CylinderTestDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
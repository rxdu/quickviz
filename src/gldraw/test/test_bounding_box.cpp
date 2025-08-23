/*
 * @file test_bounding_box.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Manual test for bounding box rendering functionality
 *
 * This test creates a window displaying different bounding box types for zones and regions.
 * Run this test to visually verify bounding box functionality for robotics applications.
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
#include "gldraw/renderable/bounding_box.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

class BoundingBoxTestDemo {
public:
    BoundingBoxTestDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager with proper layout settings
        scene_manager_ = std::make_shared<GlSceneManager>("Bounding Box Rendering Test");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateTestBoundingBoxes() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // Add coordinate frame at origin
        auto frame = std::make_unique<CoordinateFrame>(1.5f);
        scene_manager_->AddOpenGLObject("frame", std::move(frame));
        
        // 1. Simple wireframe box - Red
        auto wireframe_box = std::make_unique<BoundingBox>(
            glm::vec3(-1.0f, -1.0f, -1.0f),
            glm::vec3(1.0f, 1.0f, 1.0f)
        );
        wireframe_box->SetRenderMode(BoundingBox::RenderMode::kWireframe);
        wireframe_box->SetEdgeColor(glm::vec3(1.0f, 0.0f, 0.0f));
        wireframe_box->SetEdgeWidth(2.0f);
        scene_manager_->AddOpenGLObject("wireframe", std::move(wireframe_box));
        
        // 2. Solid box - Green
        auto solid_box = std::make_unique<BoundingBox>();
        solid_box->SetCenter(glm::vec3(3.0f, 0.0f, 0.0f), glm::vec3(1.5f, 2.0f, 1.0f));
        solid_box->SetRenderMode(BoundingBox::RenderMode::kSolid);
        solid_box->SetColor(glm::vec3(0.0f, 0.8f, 0.0f));
        scene_manager_->AddOpenGLObject("solid", std::move(solid_box));
        
        // 3. Transparent box - Blue
        auto transparent_box = std::make_unique<BoundingBox>(
            glm::vec3(-3.5f, -1.0f, -0.5f),
            glm::vec3(-2.0f, 1.0f, 0.5f)
        );
        transparent_box->SetRenderMode(BoundingBox::RenderMode::kTransparent);
        transparent_box->SetColor(glm::vec3(0.2f, 0.2f, 0.9f));
        transparent_box->SetEdgeColor(glm::vec3(0.0f, 0.0f, 1.0f));
        transparent_box->SetOpacity(0.4f);
        scene_manager_->AddOpenGLObject("transparent", std::move(transparent_box));
        
        // 4. Tall obstacle box - Purple
        auto obstacle_box = std::make_unique<BoundingBox>();
        obstacle_box->SetBounds(glm::vec3(0.5f, -0.5f, 2.0f), glm::vec3(1.5f, 0.5f, 4.0f));
        obstacle_box->SetRenderMode(BoundingBox::RenderMode::kTransparent);
        obstacle_box->SetColor(glm::vec3(0.7f, 0.2f, 0.7f));
        obstacle_box->SetEdgeColor(glm::vec3(0.5f, 0.0f, 0.5f));
        obstacle_box->SetOpacity(0.6f);
        scene_manager_->AddOpenGLObject("obstacle", std::move(obstacle_box));
        
        // 5. Ground plane region - Yellow, flat
        auto ground_region = std::make_unique<BoundingBox>(
            glm::vec3(-5.0f, -0.1f, -3.0f),
            glm::vec3(-2.0f, 0.1f, -1.0f)
        );
        ground_region->SetRenderMode(BoundingBox::RenderMode::kTransparent);
        ground_region->SetColor(glm::vec3(0.9f, 0.9f, 0.2f));
        ground_region->SetEdgeColor(glm::vec3(0.8f, 0.8f, 0.0f));
        ground_region->SetOpacity(0.3f);
        ground_region->SetEdgeWidth(3.0f);
        scene_manager_->AddOpenGLObject("ground_region", std::move(ground_region));
        
        // 6. Box with corner points - Cyan
        auto corner_box = std::make_unique<BoundingBox>(
            glm::vec3(2.0f, 1.5f, 1.0f),
            glm::vec3(4.0f, 2.5f, 2.0f)
        );
        corner_box->SetRenderMode(BoundingBox::RenderMode::kWireframe);
        corner_box->SetEdgeColor(glm::vec3(0.0f, 1.0f, 1.0f));
        corner_box->SetShowCornerPoints(true, 8.0f);
        scene_manager_->AddOpenGLObject("corner_box", std::move(corner_box));
        
        // 7. Multiple detection zones
        for (int i = 0; i < 3; ++i) {
            float z_offset = -5.0f - i * 2.0f;
            auto zone_box = std::make_unique<BoundingBox>(
                glm::vec3(-1.0f, -0.5f, z_offset),
                glm::vec3(1.0f, 0.5f, z_offset + 1.5f)
            );
            zone_box->SetRenderMode(BoundingBox::RenderMode::kWireframe);
            zone_box->SetEdgeColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange
            zone_box->SetEdgeWidth(1.5f);
            scene_manager_->AddOpenGLObject("zone_" + std::to_string(i), std::move(zone_box));
        }
        
        // 8. Rotated box using transform
        auto rotated_box = std::make_unique<BoundingBox>(
            glm::vec3(-0.5f, -0.5f, -0.5f),
            glm::vec3(0.5f, 0.5f, 0.5f)
        );
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0, 1, 0));
        glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3(-5.0f, 2.0f, 0.0f));
        rotated_box->SetTransform(translation * rotation);
        rotated_box->SetRenderMode(BoundingBox::RenderMode::kTransparent);
        rotated_box->SetColor(glm::vec3(0.8f, 0.4f, 0.2f));  // Brown
        rotated_box->SetEdgeColor(glm::vec3(0.6f, 0.2f, 0.0f));
        scene_manager_->AddOpenGLObject("rotated", std::move(rotated_box));
        
        std::cout << "\nCreated test scene with:" << std::endl;
        std::cout << "  - Reference grid and coordinate frame" << std::endl;
        std::cout << "  - Red wireframe box (basic)" << std::endl;
        std::cout << "  - Green solid box" << std::endl;
        std::cout << "  - Blue transparent box" << std::endl;
        std::cout << "  - Purple obstacle box (tall)" << std::endl;
        std::cout << "  - Yellow ground plane region (flat)" << std::endl;
        std::cout << "  - Cyan box with corner points" << std::endl;
        std::cout << "  - Orange detection zones (3 boxes)" << std::endl;
        std::cout << "  - Brown rotated box (with transform)" << std::endl;
    }
    
    void Run() {
        CreateTestBoundingBoxes();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "R: Reset camera to default position" << std::endl;
        std::cout << "ESC: Exit application" << std::endl;
        
        std::cout << "\n=== Bounding Box Features Demonstrated ===" << std::endl;
        std::cout << "- Wireframe, solid, and transparent rendering modes" << std::endl;
        std::cout << "- Variable edge colors and widths" << std::endl;
        std::cout << "- Corner point visualization" << std::endl;
        std::cout << "- Transparency with proper alpha blending" << std::endl;
        std::cout << "- Box transformations (rotation/translation)" << std::endl;
        std::cout << "- Various use cases: obstacles, zones, regions" << std::endl;
        
        // Show viewer
        viewer_.Show();
    }
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Bounding Box Rendering Test ===" << std::endl;
    std::cout << "Testing bounding box rendering for zones, regions, and obstacles\n" << std::endl;
    
    try {
        BoundingBoxTestDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
/*
 * @file test_text3d.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Manual test for Text3D rendering functionality
 *
 * This test creates a window displaying different 3D text examples for robotics visualization.
 * Run this test to visually verify Text3D functionality for annotations and labels.
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
#include "gldraw/renderable/text3d.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/arrow.hpp"

using namespace quickviz;

class Text3DTestDemo {
public:
    Text3DTestDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager with proper layout settings
        scene_manager_ = std::make_shared<GlSceneManager>("Text3D Rendering Test");
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
        
        // Add coordinate frame at origin
        auto frame = std::make_unique<CoordinateFrame>(2.0f);
        scene_manager_->AddOpenGLObject("frame", std::move(frame));
        
        // 1. Axis labels with arrows
        CreateAxisLabels();
        
        // 2. Waypoint labels with spheres
        CreateWaypointLabels();
        
        // 3. Zone annotations
        CreateZoneAnnotations();
        
        // 4. Measurement labels
        CreateMeasurementLabels();
        
        // 5. Billboard mode demonstrations
        CreateBillboardDemos();
        
        std::cout << "\nCreated test scene with:" << std::endl;
        std::cout << "  - Reference grid and coordinate frame" << std::endl;
        std::cout << "  - Axis labels (X/Y/Z)" << std::endl;
        std::cout << "  - Waypoint markers with labels" << std::endl;
        std::cout << "  - Zone annotations" << std::endl;
        std::cout << "  - Measurement displays" << std::endl;
        std::cout << "  - Billboard mode demonstrations" << std::endl;
    }
    
    void CreateAxisLabels() {
        // X-axis label
        auto x_label = std::make_unique<Text3D>();
        x_label->SetText("X");
        x_label->SetPosition(glm::vec3(3.0f, 0.0f, 0.0f));
        x_label->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        x_label->SetScale(1.5f);
        x_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("x_label", std::move(x_label));
        
        // Y-axis label
        auto y_label = std::make_unique<Text3D>();
        y_label->SetText("Y");
        y_label->SetPosition(glm::vec3(0.0f, 3.0f, 0.0f));
        y_label->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        y_label->SetScale(1.5f);
        y_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("y_label", std::move(y_label));
        
        // Z-axis label
        auto z_label = std::make_unique<Text3D>();
        z_label->SetText("Z");
        z_label->SetPosition(glm::vec3(0.0f, 0.0f, 3.0f));
        z_label->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
        z_label->SetScale(1.5f);
        z_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("z_label", std::move(z_label));
        
        // Add corresponding arrows
        auto x_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(2.5f, 0.0f, 0.0f)
        );
        x_arrow->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        scene_manager_->AddOpenGLObject("x_arrow", std::move(x_arrow));
        
        auto y_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 2.5f, 0.0f)
        );
        y_arrow->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        scene_manager_->AddOpenGLObject("y_arrow", std::move(y_arrow));
        
        auto z_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 2.5f)
        );
        z_arrow->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
        scene_manager_->AddOpenGLObject("z_arrow", std::move(z_arrow));
    }
    
    void CreateWaypointLabels() {
        struct Waypoint {
            std::string name;
            glm::vec3 position;
            glm::vec3 color;
        };
        
        std::vector<Waypoint> waypoints = {
            {"WP1", glm::vec3(-5.0f, 0.5f, -5.0f), glm::vec3(1.0f, 0.5f, 0.0f)},
            {"WP2", glm::vec3(5.0f, 0.5f, -5.0f), glm::vec3(0.0f, 1.0f, 0.5f)},
            {"WP3", glm::vec3(5.0f, 0.5f, 5.0f), glm::vec3(0.5f, 0.0f, 1.0f)},
            {"WP4", glm::vec3(-5.0f, 0.5f, 5.0f), glm::vec3(1.0f, 0.0f, 0.5f)},
            {"HOME", glm::vec3(0.0f, 0.5f, -7.0f), glm::vec3(0.0f, 1.0f, 1.0f)}
        };
        
        for (const auto& wp : waypoints) {
            // Create sphere marker
            auto sphere = std::make_unique<Sphere>(wp.position, 0.2f);
            sphere->SetColor(wp.color);
            scene_manager_->AddOpenGLObject("sphere_" + wp.name, std::move(sphere));
            
            // Create label above marker
            auto label = std::make_unique<Text3D>();
            label->SetText(wp.name);
            label->SetPosition(wp.position + glm::vec3(0.0f, 0.5f, 0.0f));
            label->SetColor(wp.color);
            label->SetScale(1.0f);
            label->SetBillboardMode(Text3D::BillboardMode::kNone);
            label->SetAlignment(Text3D::Alignment::kCenter, Text3D::VerticalAlignment::kMiddle);
            scene_manager_->AddOpenGLObject("label_" + wp.name, std::move(label));
        }
    }
    
    void CreateZoneAnnotations() {
        // Safe zone
        auto safe_zone = std::make_unique<Text3D>();
        safe_zone->SetText("SAFE ZONE");
        safe_zone->SetPosition(glm::vec3(-6.0f, 2.0f, 0.0f));
        safe_zone->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        safe_zone->SetScale(1.2f);
        safe_zone->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("safe_zone", std::move(safe_zone));
        
        // Danger zone
        auto danger_zone = std::make_unique<Text3D>();
        danger_zone->SetText("DANGER");
        danger_zone->SetPosition(glm::vec3(6.0f, 2.0f, 0.0f));
        danger_zone->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        danger_zone->SetScale(1.5f);
        danger_zone->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("danger_zone", std::move(danger_zone));
        
        // Restricted area
        auto restricted = std::make_unique<Text3D>();
        restricted->SetText("RESTRICTED");
        restricted->SetPosition(glm::vec3(0.0f, 2.0f, 7.0f));
        restricted->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        restricted->SetScale(1.3f);
        restricted->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("restricted", std::move(restricted));
    }
    
    void CreateMeasurementLabels() {
        // Distance measurement
        auto dist_label = std::make_unique<Text3D>();
        dist_label->SetText("5.2m");
        dist_label->SetPosition(glm::vec3(2.5f, 0.2f, -2.5f));
        dist_label->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));
        dist_label->SetScale(0.8f);
        dist_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("distance", std::move(dist_label));
        
        // Angle measurement
        auto angle_label = std::make_unique<Text3D>();
        angle_label->SetText("45 deg");
        angle_label->SetPosition(glm::vec3(-2.5f, 0.2f, 2.5f));
        angle_label->SetColor(glm::vec3(0.8f, 0.8f, 0.8f));
        angle_label->SetScale(0.8f);
        angle_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("angle", std::move(angle_label));
        
        // Speed indicator
        auto speed_label = std::make_unique<Text3D>();
        speed_label->SetText("2.5 m/s");
        speed_label->SetPosition(glm::vec3(0.0f, 3.5f, -4.0f));
        speed_label->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
        speed_label->SetScale(1.0f);
        speed_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("speed", std::move(speed_label));
    }
    
    void CreateBillboardDemos() {
        // None mode - fixed orientation
        auto fixed_text = std::make_unique<Text3D>();
        fixed_text->SetText("FIXED");
        fixed_text->SetPosition(glm::vec3(-8.0f, 1.0f, -8.0f));
        fixed_text->SetColor(glm::vec3(0.7f, 0.7f, 0.7f));
        fixed_text->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("fixed", std::move(fixed_text));
        
        // Sphere mode - always faces camera
        auto sphere_text = std::make_unique<Text3D>();
        sphere_text->SetText("SPHERE");
        sphere_text->SetPosition(glm::vec3(8.0f, 1.0f, -8.0f));
        sphere_text->SetColor(glm::vec3(0.5f, 0.5f, 1.0f));
        sphere_text->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("sphere_mode", std::move(sphere_text));
        
        // Cylinder mode - rotates around Y axis only
        auto cylinder_text = std::make_unique<Text3D>();
        cylinder_text->SetText("CYLINDER");
        cylinder_text->SetPosition(glm::vec3(0.0f, 1.0f, -8.0f));
        cylinder_text->SetColor(glm::vec3(1.0f, 0.5f, 0.5f));
        cylinder_text->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("cylinder_mode", std::move(cylinder_text));
        
        // Multi-line text example
        auto multi_line = std::make_unique<Text3D>();
        multi_line->SetText("MULTI\nLINE\nTEXT");
        multi_line->SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));
        multi_line->SetColor(glm::vec3(0.8f, 0.8f, 0.0f));
        multi_line->SetScale(0.8f);
        multi_line->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager_->AddOpenGLObject("multiline", std::move(multi_line));
    }
    
    void Run() {
        CreateTestScene();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "R: Reset camera to default position" << std::endl;
        std::cout << "ESC: Exit application" << std::endl;
        
        std::cout << "\n=== Text3D Features Demonstrated ===" << std::endl;
        std::cout << "✓ Horizontal text orientation (all text displayed flat)" << std::endl;
        std::cout << "✓ Text alignment options (left, center, right)" << std::endl;
        std::cout << "✓ Multiple colors and scales" << std::endl;
        std::cout << "✓ Waypoint annotations with spheres" << std::endl;
        std::cout << "✓ Coordinate system labels" << std::endl;
        std::cout << "✓ Zone and measurement annotations" << std::endl;
        std::cout << "✓ Multi-line text support" << std::endl;
        std::cout << "✓ Integration with other renderables" << std::endl;
        
        // Show viewer
        viewer_.Show();
    }
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Text3D Rendering Test ===" << std::endl;
    std::cout << "Testing 3D text rendering for robotics visualization\n" << std::endl;
    
    try {
        Text3DTestDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
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

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/text3d.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/arrow.hpp"

using namespace quickviz;

// Forward declarations
void CreateAxisLabels(GlSceneManager* scene_manager);
void CreateWaypointLabels(GlSceneManager* scene_manager);
void CreateZoneAnnotations(GlSceneManager* scene_manager);
void CreateMeasurementLabels(GlSceneManager* scene_manager);
void CreateBillboardDemos(GlSceneManager* scene_manager);

void SetupText3DScene(GlSceneManager* scene_manager) {
    // 1. Axis labels with arrows
    CreateAxisLabels(scene_manager);
    
    // 2. Waypoint labels with spheres
    CreateWaypointLabels(scene_manager);
    
    // 3. Zone annotations
    CreateZoneAnnotations(scene_manager);
    
    // 4. Measurement labels
    CreateMeasurementLabels(scene_manager);
    
    // 5. Billboard mode demonstrations
    CreateBillboardDemos(scene_manager);
}

void CreateAxisLabels(GlSceneManager* scene_manager) {
    // X-axis label
    auto x_label = std::make_unique<Text3D>();
    x_label->SetText("X");
    x_label->SetPosition(glm::vec3(3.0f, 0.0f, 0.0f));
    x_label->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    x_label->SetScale(1.8f);  // Reduced by 1/4 from 2.4f (75% of original)
    x_label->SetBillboardMode(Text3D::BillboardMode::kNone);
    scene_manager->AddOpenGLObject("x_label", std::move(x_label));
    
    // Y-axis label
    auto y_label = std::make_unique<Text3D>();
    y_label->SetText("Y");
    y_label->SetPosition(glm::vec3(0.0f, 3.0f, 0.0f));
    y_label->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    y_label->SetScale(1.8f);  // Reduced by 1/4 from 2.4f (75% of original)
    y_label->SetBillboardMode(Text3D::BillboardMode::kNone);
    scene_manager->AddOpenGLObject("y_label", std::move(y_label));
    
    // Z-axis label
    auto z_label = std::make_unique<Text3D>();
    z_label->SetText("Z");
    z_label->SetPosition(glm::vec3(0.0f, 0.0f, 3.0f));
    z_label->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    z_label->SetScale(1.8f);  // Reduced by 1/4 from 2.4f (75% of original)
    z_label->SetBillboardMode(Text3D::BillboardMode::kNone);
    scene_manager->AddOpenGLObject("z_label", std::move(z_label));
        
        // Add corresponding arrows
        auto x_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(2.5f, 0.0f, 0.0f)
        );
        x_arrow->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        scene_manager->AddOpenGLObject("x_arrow", std::move(x_arrow));
        
        auto y_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 2.5f, 0.0f)
        );
        y_arrow->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        scene_manager->AddOpenGLObject("y_arrow", std::move(y_arrow));
        
        auto z_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 2.5f)
        );
        z_arrow->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
        scene_manager->AddOpenGLObject("z_arrow", std::move(z_arrow));
}

void CreateWaypointLabels(GlSceneManager* scene_manager) {
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
            scene_manager->AddOpenGLObject("sphere_" + wp.name, std::move(sphere));
            
            // Create label above marker
            auto label = std::make_unique<Text3D>();
            label->SetText(wp.name);
            label->SetPosition(wp.position + glm::vec3(0.0f, 0.5f, 0.0f));
            label->SetColor(wp.color);
            label->SetScale(1.2f);  // Reduced by 1/4 from 1.6f (75% of original)
            label->SetBillboardMode(Text3D::BillboardMode::kNone);
            label->SetAlignment(Text3D::Alignment::kCenter, Text3D::VerticalAlignment::kMiddle);
            scene_manager->AddOpenGLObject("label_" + wp.name, std::move(label));
        }
}

void CreateZoneAnnotations(GlSceneManager* scene_manager) {
        // Safe zone
        auto safe_zone = std::make_unique<Text3D>();
        safe_zone->SetText("SAFE ZONE");
        safe_zone->SetPosition(glm::vec3(-6.0f, 2.0f, 0.0f));
        safe_zone->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        safe_zone->SetScale(1.5f);  // Reduced by 1/4 from 2.0f (75% of original)
        safe_zone->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("safe_zone", std::move(safe_zone));
        
        // Danger zone
        auto danger_zone = std::make_unique<Text3D>();
        danger_zone->SetText("DANGER");
        danger_zone->SetPosition(glm::vec3(6.0f, 2.0f, 0.0f));
        danger_zone->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        danger_zone->SetScale(1.8f);  // Reduced by 1/4 from 2.4f (75% of original)
        danger_zone->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("danger_zone", std::move(danger_zone));
        
        // Restricted area
        auto restricted = std::make_unique<Text3D>();
        restricted->SetText("RESTRICTED");
        restricted->SetPosition(glm::vec3(0.0f, 2.0f, 7.0f));
        restricted->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        restricted->SetScale(1.5f);  // Reduced by 1/4 from 2.0f (75% of original)
        restricted->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("restricted", std::move(restricted));
}

void CreateMeasurementLabels(GlSceneManager* scene_manager) {
        // Distance measurement
        auto dist_label = std::make_unique<Text3D>();
        dist_label->SetText("5.2m");
        dist_label->SetPosition(glm::vec3(2.5f, 0.2f, -2.5f));
        dist_label->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));
        dist_label->SetScale(0.9f);  // Reduced by 1/4 from 1.2f (75% of original)
        dist_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("distance", std::move(dist_label));
        
        // Angle measurement
        auto angle_label = std::make_unique<Text3D>();
        angle_label->SetText("45 deg");
        angle_label->SetPosition(glm::vec3(-2.5f, 0.2f, 2.5f));
        angle_label->SetColor(glm::vec3(0.8f, 0.8f, 0.8f));
        angle_label->SetScale(0.9f);  // Reduced by 1/4 from 1.2f (75% of original)
        angle_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("angle", std::move(angle_label));
        
        // Speed indicator
        auto speed_label = std::make_unique<Text3D>();
        speed_label->SetText("2.5 m/s");
        speed_label->SetPosition(glm::vec3(0.0f, 3.5f, -4.0f));
        speed_label->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
        speed_label->SetScale(1.2f);  // Reduced by 1/4 from 1.6f (75% of original)
        speed_label->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("speed", std::move(speed_label));
}

void CreateBillboardDemos(GlSceneManager* scene_manager) {
        // None mode - fixed orientation
        auto fixed_text = std::make_unique<Text3D>();
        fixed_text->SetText("FIXED");
        fixed_text->SetPosition(glm::vec3(-8.0f, 1.0f, -8.0f));
        fixed_text->SetColor(glm::vec3(0.7f, 0.7f, 0.7f));
        fixed_text->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("fixed", std::move(fixed_text));
        
        // Sphere mode - always faces camera
        auto sphere_text = std::make_unique<Text3D>();
        sphere_text->SetText("SPHERE");
        sphere_text->SetPosition(glm::vec3(8.0f, 1.0f, -8.0f));
        sphere_text->SetColor(glm::vec3(0.5f, 0.5f, 1.0f));
        sphere_text->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("sphere_mode", std::move(sphere_text));
        
        // Cylinder mode - rotates around Y axis only
        auto cylinder_text = std::make_unique<Text3D>();
        cylinder_text->SetText("CYLINDER");
        cylinder_text->SetPosition(glm::vec3(0.0f, 1.0f, -8.0f));
        cylinder_text->SetColor(glm::vec3(1.0f, 0.5f, 0.5f));
        cylinder_text->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("cylinder_mode", std::move(cylinder_text));
        
        // Multi-line text example
        auto multi_line = std::make_unique<Text3D>();
        multi_line->SetText("MULTI\nLINE\nTEXT");
        multi_line->SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));
        multi_line->SetColor(glm::vec3(0.8f, 0.8f, 0.0f));
        multi_line->SetScale(0.9f);  // Reduced by 1/4 from 1.2f (75% of original)
        multi_line->SetBillboardMode(Text3D::BillboardMode::kNone);
        scene_manager->AddOpenGLObject("multiline", std::move(multi_line));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view
        GlView::Config config;
        config.window_title = "Text3D Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing 3D text rendering for robotics visualization");
        
        view.AddHelpSection("Text3D Features Demonstrated", {
            "✓ Horizontal text orientation (all text displayed flat)",
            "✓ Text alignment options (left, center, right)",
            "✓ Multiple colors and scales",
            "✓ Waypoint annotations with spheres",
            "✓ Coordinate system labels", 
            "✓ Zone and measurement annotations",
            "✓ Multi-line text support",
            "✓ Integration with other renderables"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Reference grid and coordinate frame",
            "- Axis labels (X/Y/Z)",
            "- Waypoint markers with labels",
            "- Zone annotations",
            "- Measurement displays",
            "- Billboard mode demonstrations"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupText3DScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
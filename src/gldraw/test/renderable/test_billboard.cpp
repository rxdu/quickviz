/**
 * @file test_billboard.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for Billboard primitive with ImGui font integration
 *
 * This test demonstrates the modern Billboard primitive as a replacement
 * for the primitive Text3D implementation, showcasing:
 * - High-quality ImGui font rendering
 * - Full Unicode support
 * - Multiple billboard modes
 * - Selection support via GeometricPrimitive base
 * - Professional typography and visual effects
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>

#include "gldraw/gl_viewer.hpp"
#include "gldraw/renderable/billboard.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/arrow.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

// Forward declarations
void CreateAxisLabels(GlSceneManager* scene_manager);
void CreateWaypointLabels(GlSceneManager* scene_manager);
void CreateBillboardModes(GlSceneManager* scene_manager);
void CreateTypographyDemo(GlSceneManager* scene_manager);
void CreateSelectionDemo(GlSceneManager* scene_manager);

void SetupBillboardScene(GlSceneManager* scene_manager) {
    std::cout << "=== Billboard Primitive Test ===" << std::endl;
    std::cout << "Modern Billboard primitive replacing primitive Text3D implementation" << std::endl;
    
    // Add reference grid
    auto grid = std::make_unique<Grid>();
    grid->SetSize(20.0f);
    grid->SetColor(glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // 1. Axis labels with professional typography
    CreateAxisLabels(scene_manager);
    
    // 2. Waypoint labels demonstrating selection
    CreateWaypointLabels(scene_manager);
    
    // 3. Billboard mode demonstrations
    CreateBillboardModes(scene_manager);
    
    // 4. Typography and visual effects demo
    CreateTypographyDemo(scene_manager);
    
    // 5. Selection and interaction demo
    CreateSelectionDemo(scene_manager);
    
    std::cout << "✓ Billboard scene setup complete!" << std::endl;
    std::cout << "✓ Features: ImGui fonts, Unicode support, selection, visual effects" << std::endl;
}

void CreateAxisLabels(GlSceneManager* scene_manager) {
    // X-axis label - Red
    auto x_label = std::make_unique<Billboard>("X-Axis");
    x_label->SetPosition(glm::vec3(4.0f, 0.0f, 0.0f));
    x_label->SetColor(glm::vec3(1.0f, 0.2f, 0.2f));
    x_label->SetFontSize(20.0f);
    x_label->SetBillboardMode(Billboard::Mode::kSphere);
    x_label->SetAlignment(Billboard::Alignment::kLeft);
    scene_manager->AddOpenGLObject("x_axis_label", std::move(x_label));
    
    // Y-axis label - Green  
    auto y_label = std::make_unique<Billboard>("Y-Axis");
    y_label->SetPosition(glm::vec3(0.0f, 4.0f, 0.0f));
    y_label->SetColor(glm::vec3(0.2f, 1.0f, 0.2f));
    y_label->SetFontSize(20.0f);
    y_label->SetBillboardMode(Billboard::Mode::kSphere);
    y_label->SetAlignment(Billboard::Alignment::kLeft);
    scene_manager->AddOpenGLObject("y_axis_label", std::move(y_label));
    
    // Z-axis label - Blue
    auto z_label = std::make_unique<Billboard>("Z-Axis");
    z_label->SetPosition(glm::vec3(0.0f, 0.0f, 4.0f));
    z_label->SetColor(glm::vec3(0.2f, 0.2f, 1.0f));
    z_label->SetFontSize(20.0f);
    z_label->SetBillboardMode(Billboard::Mode::kSphere);
    z_label->SetAlignment(Billboard::Alignment::kLeft);
    scene_manager->AddOpenGLObject("z_axis_label", std::move(z_label));
    
    // Add corresponding arrows
    auto x_arrow = std::make_unique<Arrow>(glm::vec3(0.0f), glm::vec3(3.5f, 0.0f, 0.0f));
    x_arrow->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("x_arrow", std::move(x_arrow));
    
    auto y_arrow = std::make_unique<Arrow>(glm::vec3(0.0f), glm::vec3(0.0f, 3.5f, 0.0f));
    y_arrow->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("y_arrow", std::move(y_arrow));
    
    auto z_arrow = std::make_unique<Arrow>(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 3.5f));
    z_arrow->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    scene_manager->AddOpenGLObject("z_arrow", std::move(z_arrow));
    
    std::cout << "✓ Created axis labels with professional typography" << std::endl;
}

void CreateWaypointLabels(GlSceneManager* scene_manager) {
    struct Waypoint {
        std::string name;
        glm::vec3 position;
        glm::vec3 color;
    };
    
    std::vector<Waypoint> waypoints = {
        {"Start", glm::vec3(-6.0f, -3.0f, 1.0f), glm::vec3(0.2f, 0.8f, 0.2f)},
        {"Checkpoint A", glm::vec3(-2.0f, 2.0f, 2.0f), glm::vec3(0.8f, 0.6f, 0.2f)},
        {"Checkpoint B", glm::vec3(3.0f, -1.0f, 1.5f), glm::vec3(0.6f, 0.2f, 0.8f)},
        {"Goal", glm::vec3(6.0f, 3.0f, 2.5f), glm::vec3(0.8f, 0.2f, 0.2f)}
    };
    
    for (const auto& wp : waypoints) {
        // Create sphere marker
        auto sphere = std::make_unique<Sphere>();
        sphere->SetCenter(wp.position);
        sphere->SetRadius(0.3f);
        sphere->SetColor(wp.color);
        std::string sphere_name = "waypoint_sphere_" + wp.name;
        std::replace(sphere_name.begin(), sphere_name.end(), ' ', '_');
        scene_manager->AddOpenGLObject(sphere_name, std::move(sphere));
        
        // Create billboard label
        auto label = std::make_unique<Billboard>(wp.name);
        label->SetPosition(wp.position + glm::vec3(0.0f, 0.0f, 0.8f));
        label->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));
        label->SetFontSize(16.0f);
        label->SetBillboardMode(Billboard::Mode::kSphere);
        label->SetAlignment(Billboard::Alignment::kCenter);
        label->SetBackgroundEnabled(true);
        label->SetBackgroundColor(glm::vec4(0.0f, 0.0f, 0.0f, 0.7f));
        label->SetBackgroundPadding(6.0f);
        std::string label_name = "waypoint_label_" + wp.name;
        std::replace(label_name.begin(), label_name.end(), ' ', '_');
        scene_manager->AddOpenGLObject(label_name, std::move(label));
    }
    
    std::cout << "✓ Created waypoint labels with selection support" << std::endl;
}

void CreateBillboardModes(GlSceneManager* scene_manager) {
    // Sphere mode - always face camera
    auto sphere_label = std::make_unique<Billboard>("Sphere Mode\n(Always faces camera)");
    sphere_label->SetPosition(glm::vec3(-8.0f, 5.0f, 0.0f));
    sphere_label->SetColor(glm::vec3(1.0f, 0.8f, 0.2f));
    sphere_label->SetFontSize(14.0f);
    sphere_label->SetBillboardMode(Billboard::Mode::kSphere);
    sphere_label->SetAlignment(Billboard::Alignment::kCenter);
    sphere_label->SetBackgroundEnabled(true);
    sphere_label->SetBackgroundColor(glm::vec4(0.2f, 0.1f, 0.0f, 0.8f));
    scene_manager->AddOpenGLObject("sphere_mode_label", std::move(sphere_label));
    
    // Cylinder mode - only horizontal rotation
    auto cylinder_label = std::make_unique<Billboard>("Cylinder Mode\n(Horizontal rotation only)");
    cylinder_label->SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));
    cylinder_label->SetColor(glm::vec3(0.2f, 0.8f, 1.0f));
    cylinder_label->SetFontSize(14.0f);
    cylinder_label->SetBillboardMode(Billboard::Mode::kCylinder);
    cylinder_label->SetAlignment(Billboard::Alignment::kCenter);
    cylinder_label->SetBackgroundEnabled(true);
    cylinder_label->SetBackgroundColor(glm::vec4(0.0f, 0.2f, 0.3f, 0.8f));
    scene_manager->AddOpenGLObject("cylinder_mode_label", std::move(cylinder_label));
    
    // Fixed mode - no rotation
    auto fixed_label = std::make_unique<Billboard>("Fixed Mode\n(No billboarding)");
    fixed_label->SetPosition(glm::vec3(8.0f, 5.0f, 0.0f));
    fixed_label->SetColor(glm::vec3(1.0f, 0.2f, 0.8f));
    fixed_label->SetFontSize(14.0f);
    fixed_label->SetBillboardMode(Billboard::Mode::kFixed);
    fixed_label->SetAlignment(Billboard::Alignment::kCenter);
    fixed_label->SetBackgroundEnabled(true);
    fixed_label->SetBackgroundColor(glm::vec4(0.3f, 0.0f, 0.2f, 0.8f));
    scene_manager->AddOpenGLObject("fixed_mode_label", std::move(fixed_label));
    
    std::cout << "✓ Created billboard mode demonstrations" << std::endl;
}

void CreateTypographyDemo(GlSceneManager* scene_manager) {
    // Large title
    auto title = std::make_unique<Billboard>("Billboard Typography Demo");
    title->SetPosition(glm::vec3(0.0f, -6.0f, 2.0f));
    title->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));
    title->SetFontSize(24.0f);
    title->SetBillboardMode(Billboard::Mode::kSphere);
    title->SetAlignment(Billboard::Alignment::kCenter);
    title->SetOutlineEnabled(true);
    title->SetOutlineColor(glm::vec3(0.0f, 0.0f, 0.0f));
    title->SetOutlineWidth(2.0f);
    scene_manager->AddOpenGLObject("typography_title", std::move(title));
    
    // Small sizes demonstration
    std::vector<float> sizes = {10.0f, 14.0f, 18.0f, 22.0f, 28.0f};
    for (size_t i = 0; i < sizes.size(); ++i) {
        auto size_demo = std::make_unique<Billboard>("Size " + std::to_string((int)sizes[i]) + "px");
        size_demo->SetPosition(glm::vec3(-6.0f + i * 3.0f, -8.0f, 0.0f));
        size_demo->SetColor(glm::vec3(0.8f + i * 0.05f, 0.6f, 0.8f - i * 0.1f));
        size_demo->SetFontSize(sizes[i]);
        size_demo->SetBillboardMode(Billboard::Mode::kSphere);
        size_demo->SetAlignment(Billboard::Alignment::kCenter);
        std::string name = "size_demo_" + std::to_string(i);
        scene_manager->AddOpenGLObject(name, std::move(size_demo));
    }
    
    // Alignment demonstration
    std::vector<std::pair<Billboard::Alignment, std::string>> alignments = {
        {Billboard::Alignment::kLeft, "Left Aligned"},
        {Billboard::Alignment::kCenter, "Center Aligned"},
        {Billboard::Alignment::kRight, "Right Aligned"}
    };
    
    for (size_t i = 0; i < alignments.size(); ++i) {
        auto align_demo = std::make_unique<Billboard>(alignments[i].second);
        align_demo->SetPosition(glm::vec3(-3.0f + i * 3.0f, -10.0f, 0.0f));
        align_demo->SetColor(glm::vec3(0.9f, 0.9f, 0.3f));
        align_demo->SetFontSize(16.0f);
        align_demo->SetBillboardMode(Billboard::Mode::kSphere);
        align_demo->SetAlignment(alignments[i].first);
        std::string name = "alignment_demo_" + std::to_string(i);
        scene_manager->AddOpenGLObject(name, std::move(align_demo));
    }
    
    std::cout << "✓ Created typography and effects demonstrations" << std::endl;
}

void CreateSelectionDemo(GlSceneManager* scene_manager) {
    // Create selectable billboards arranged in a grid
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            auto selectable = std::make_unique<Billboard>("Click Me\n(" + std::to_string(i) + "," + std::to_string(j) + ")");
            selectable->SetPosition(glm::vec3(-8.0f + i * 4.0f, -3.0f + j * 2.0f, 3.0f));
            selectable->SetColor(glm::vec3(0.7f + i * 0.1f, 0.7f + j * 0.1f, 0.8f));
            selectable->SetFontSize(14.0f);
            selectable->SetBillboardMode(Billboard::Mode::kSphere);
            selectable->SetAlignment(Billboard::Alignment::kCenter);
            selectable->SetBackgroundEnabled(true);
            selectable->SetBackgroundColor(glm::vec4(0.1f, 0.1f, 0.2f, 0.6f));
            selectable->SetBackgroundPadding(4.0f);
            
            std::string name = "selectable_billboard_" + std::to_string(i) + "_" + std::to_string(j);
            scene_manager->AddOpenGLObject(name, std::move(selectable));
        }
    }
    
    std::cout << "✓ Created selectable billboard grid (9 items)" << std::endl;
}

int main(int argc, char* argv[]) {
    try {
        std::cout << "=== Billboard Primitive Test ===" << std::endl;
        std::cout << "Testing modern Billboard primitive as Text3D replacement" << std::endl;
        std::cout << std::endl;
        
        // Configure the viewer
        GlViewer::Config config;
        config.window_title = "Billboard Primitive Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create viewer
        GlViewer viewer(config);
        
        // Set up description and help sections
        viewer.SetDescription("Testing Billboard primitive with ImGui font integration as Text3D replacement");
        
        viewer.AddHelpSection("Features Demonstrated", {
            "✓ ImGui font system integration",
            "✓ Professional typography rendering", 
            "✓ Multiple billboard modes (sphere/cylinder/fixed)",
            "✓ Selection support via GeometricPrimitive",
            "✓ Visual effects (backgrounds, outlines)",
            "✓ Text alignment and sizing options"
        });
        
        viewer.AddHelpSection("Billboard Test Scene", {
            "- Axis labels with professional typography",
            "- Waypoint labels with selection support",
            "- Billboard mode demonstrations", 
            "- Typography and effects demo",
            "- Interactive selection grid (9 billboards)"
        });
        
        viewer.AddHelpSection("Selection Testing", {
            "Left Click on Billboard: Select (highlight effect)",
            "Ctrl+Click: Multi-selection",
            "Selection shows material-based highlighting"
        });
        
        // Set the scene setup callback
        viewer.SetSceneSetup(SetupBillboardScene);
        
        std::cout << "✓ Test setup complete! Starting interactive session..." << std::endl;
        std::cout << "Click on billboards to test selection functionality!" << std::endl;
        
        // Run the viewer
        viewer.Run();
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Billboard test failed: " << e.what() << std::endl;
        return 1;
    }
}
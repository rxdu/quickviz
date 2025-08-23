/**
 * @file test_coordinate_frame.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief Test for the CoordinateFrame class
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <memory>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

void SetupCoordinateFrameScene(GlSceneManager* scene_manager) {
    // Add a grid for reference
    auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));

    // Add main coordinate frame at origin
    auto coord_frame_main = std::make_unique<CoordinateFrame>(3.0f, false);
    scene_manager->AddOpenGLObject("coordinate_frame_main", std::move(coord_frame_main));

    // Create a coordinate frame rotated 45 degrees around Y axis
    auto coord_frame_rotated_y = std::make_unique<CoordinateFrame>(2.0f, false);
    glm::quat rotation_y = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    coord_frame_rotated_y->SetPose(glm::vec3(5.0f, 0.0f, 5.0f), rotation_y);
    scene_manager->AddOpenGLObject("coord_frame_rotated_y", std::move(coord_frame_rotated_y));

    // Create a coordinate frame rotated 45 degrees around X axis
    auto coord_frame_rotated_x = std::make_unique<CoordinateFrame>(2.0f, false);
    glm::quat rotation_x = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    coord_frame_rotated_x->SetPose(glm::vec3(-5.0f, 0.0f, 5.0f), rotation_x);
    scene_manager->AddOpenGLObject("coord_frame_rotated_x", std::move(coord_frame_rotated_x));

    // Create a coordinate frame rotated 45 degrees around Z axis
    auto coord_frame_rotated_z = std::make_unique<CoordinateFrame>(2.0f, false);
    glm::quat rotation_z = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    coord_frame_rotated_z->SetPose(glm::vec3(0.0f, 0.0f, -5.0f), rotation_z);
    scene_manager->AddOpenGLObject("coord_frame_rotated_z", std::move(coord_frame_rotated_z));

    // Small coordinate frames to show different scales
    auto small_frame_1 = std::make_unique<CoordinateFrame>(1.0f, false);
    small_frame_1->SetPose(glm::vec3(3.0f, 3.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("small_frame_1", std::move(small_frame_1));

    auto small_frame_2 = std::make_unique<CoordinateFrame>(1.0f, false);
    small_frame_2->SetPose(glm::vec3(-3.0f, 3.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("small_frame_2", std::move(small_frame_2));

    // Large coordinate frame
    auto large_frame = std::make_unique<CoordinateFrame>(5.0f, false);
    large_frame->SetPose(glm::vec3(0.0f, 0.0f, 8.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("large_frame", std::move(large_frame));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Coordinate Frame Rendering Test";
        config.coordinate_frame_size = 2.0f;
        config.show_coordinate_frame = false;  // We'll add our own coordinate frames
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing coordinate frame rendering with various orientations and scales");
        
        view.AddHelpSection("Coordinate Frame Features Demonstrated", {
            "- Main coordinate frame at origin (3.0f scale)",
            "- Rotated frames around X, Y, and Z axes (45° each)",
            "- Different scales: small (1.0f), main (3.0f), large (5.0f)",
            "- Various positions to show transformation capabilities",
            "- Grid reference for spatial understanding"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Reference grid (10x10 units)",
            "- Main coordinate frame at origin (Red=X, Green=Y, Blue=Z)",
            "- Y-rotated frame at (5, 0, 5)",
            "- X-rotated frame at (-5, 0, 5)",
            "- Z-rotated frame at (0, 0, -5)",
            "- Small frames at (±3, 3, 0) for scale comparison",
            "- Large frame at (0, 0, 8) for scale demonstration"
        });
        
        view.AddHelpSection("Coordinate Frame Convention", {
            "- Red arrow: X-axis (right/forward)",
            "- Green arrow: Y-axis (up/left)",
            "- Blue arrow: Z-axis (out/up)",
            "- Right-hand coordinate system",
            "- Arrow length indicates scale"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupCoordinateFrameScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
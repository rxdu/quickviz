/*
 * @file test_pose.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for Pose rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/pose.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

void SetupPoseScene(GlSceneManager* scene_manager) {
    // Add grid for reference
    auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // 1. Simple pose at origin - no trail
    auto pose1 = std::make_unique<Pose>();
    pose1->SetPose(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    pose1->SetAxisLength(1.5f);
    pose1->SetAxisColors(glm::vec3(1.0f, 0.3f, 0.3f),   // Bright red X
                         glm::vec3(0.3f, 1.0f, 0.3f),   // Bright green Y  
                         glm::vec3(0.3f, 0.3f, 1.0f));  // Bright blue Z
    pose1->SetAxisWidth(3.0f);
    scene_manager->AddOpenGLObject("pose_origin", std::move(pose1));
    
    // 2. Rotated pose with line trail
    auto pose2 = std::make_unique<Pose>();
    pose2->SetPose(glm::vec3(3.0f, 0.0f, 0.0f), 
                   glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
    pose2->SetAxisLength(1.2f);
    pose2->SetTrailMode(Pose::TrailMode::kLine);
    pose2->SetTrailColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Yellow trail
    pose2->SetTrailWidth(2.5f);
    pose2->SetTrailLength(20);
    
    // Simulate movement for trail
    for (int i = 1; i <= 15; ++i) {
        float t = i * 0.2f;
        glm::vec3 pos = glm::vec3(3.0f + std::sin(t) * 2.0f, std::cos(t) * 1.5f, std::sin(t * 0.5f) * 0.8f);
        glm::quat rot = glm::angleAxis(glm::radians(45.0f + t * 30.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        pose2->SetPose(pos, rot);
    }
    scene_manager->AddOpenGLObject("pose_trail_line", std::move(pose2));
    
    // 3. Animated pose with dots trail
    auto pose3 = std::make_unique<Pose>();
    pose3->SetPose(glm::vec3(-3.0f, 0.0f, 1.0f), 
                   glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    pose3->SetAxisLength(0.8f);
    pose3->SetTrailMode(Pose::TrailMode::kDots);
    pose3->SetTrailColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange trail
    pose3->SetTrailWidth(4.0f);
    pose3->SetTrailLength(12);
    
    // Simulate circular motion for dots trail
    for (int i = 1; i <= 10; ++i) {
        float angle = i * 0.6f;
        glm::vec3 pos = glm::vec3(-3.0f + std::cos(angle) * 1.5f, 
                                  std::sin(angle) * 1.5f, 
                                  1.0f + std::sin(angle * 2.0f) * 0.5f);
        glm::quat rot = glm::angleAxis(glm::radians(90.0f + angle * 20.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        pose3->SetPose(pos, rot);
    }
    scene_manager->AddOpenGLObject("pose_trail_dots", std::move(pose3));
    
    // 4. Large pose with custom colors - elevated position
    auto pose4 = std::make_unique<Pose>();
    pose4->SetPose(glm::vec3(0.0f, 3.0f, 2.0f), 
                   glm::angleAxis(glm::radians(30.0f), glm::vec3(1.0f, 1.0f, 0.0f)));
    pose4->SetAxisLength(2.5f);
    pose4->SetAxisColors(glm::vec3(1.0f, 0.0f, 1.0f),   // Magenta X
                         glm::vec3(0.0f, 1.0f, 1.0f),   // Cyan Y
                         glm::vec3(1.0f, 1.0f, 0.0f));  // Yellow Z
    pose4->SetAxisWidth(4.0f);
    pose4->SetScale(1.5f);
    scene_manager->AddOpenGLObject("pose_large", std::move(pose4));
    
    // 5. Semi-transparent pose with fading trail
    auto pose5 = std::make_unique<Pose>();
    pose5->SetPose(glm::vec3(1.5f, -2.5f, 0.5f), 
                   glm::angleAxis(glm::radians(60.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    pose5->SetAxisLength(1.0f);
    pose5->SetTransparency(0.7f);
    pose5->SetTrailMode(Pose::TrailMode::kFading);
    pose5->SetTrailColor(glm::vec3(0.8f, 0.2f, 0.8f));  // Purple trail
    pose5->SetTrailWidth(3.0f);
    pose5->SetTrailFadeTime(5.0f);
    pose5->SetTrailLength(25);
    
    // Simulate figure-8 motion for fading trail
    for (int i = 1; i <= 20; ++i) {
        float t = i * 0.3f;
        glm::vec3 pos = glm::vec3(1.5f + std::sin(t) * 1.2f, 
                                  -2.5f + std::sin(t * 2.0f) * 0.8f, 
                                  0.5f + std::cos(t) * 0.3f);
        glm::quat rot = glm::angleAxis(glm::radians(60.0f + t * 15.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        pose5->SetPose(pos, rot);
    }
    scene_manager->AddOpenGLObject("pose_fading", std::move(pose5));
    
    // 6. Small pose without frame (trail only)
    auto pose6 = std::make_unique<Pose>();
    pose6->SetPose(glm::vec3(-1.5f, -1.5f, -0.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    pose6->SetShowFrame(false);  // Hide coordinate frame
    pose6->SetTrailMode(Pose::TrailMode::kLine);
    pose6->SetTrailColor(glm::vec3(0.2f, 0.8f, 0.8f));  // Teal trail
    pose6->SetTrailWidth(2.0f);
    pose6->SetTrailLength(15);
    
    // Simulate spiral motion (trail only, no frame)
    for (int i = 1; i <= 12; ++i) {
        float t = i * 0.4f;
        float radius = 0.8f * (1.0f - i / 15.0f);  // Shrinking spiral
        glm::vec3 pos = glm::vec3(-1.5f + std::cos(t * 3.0f) * radius, 
                                  -1.5f + std::sin(t * 3.0f) * radius, 
                                  -0.5f + t * 0.1f);
        pose6->SetPose(pos, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    }
    scene_manager->AddOpenGLObject("pose_trail_only", std::move(pose6));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Pose Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing 6-DOF pose visualization with coordinate frames and history trails");
        
        view.AddHelpSection("Pose Features Demonstrated", {
            "- 6-DOF pose visualization (position + orientation)",
            "- Customizable coordinate frame with axis colors and lengths",
            "- Multiple trail modes: line, dots, fading, arrows",  
            "- Trail history with configurable length and colors",
            "- Scale and transparency control",
            "- Frame-only or trail-only visualization modes"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Origin pose (0,0,0): Basic coordinate frame with bright axis colors",
            "- Trail line pose (3,0,0): Yellow line trail showing sinusoidal motion",
            "- Trail dots pose (-3,0,1): Orange dot trail showing circular motion", 
            "- Large pose (0,3,2): Scaled frame with custom magenta/cyan/yellow colors",
            "- Fading trail pose (1.5,-2.5,0.5): Purple trail with time-based fading",
            "- Trail-only pose (-1.5,-1.5,-0.5): Teal spiral trail without coordinate frame"
        });
        
        view.AddHelpSection("Robotics Applications", {
            "- Current robot pose visualization",
            "- Goal pose and waypoint display", 
            "- Path planning result visualization",
            "- Transform tree (tf) visualization",
            "- 6-DOF manipulation target display",
            "- Multi-robot coordination and formation",
            "- SLAM trajectory and loop closure visualization"
        });
        
        view.AddHelpSection("Trail Mode Details", {
            "- kLine: Connected line segments showing continuous path",
            "- kDots: Discrete points at each pose update",
            "- kFading: Line trail with time-based alpha decay", 
            "- kArrows: Small orientation indicators (future enhancement)",
            "- Configurable trail length, color, width, and fade time",
            "- Real-time trail updates with pose changes"
        });
        
        view.AddHelpSection("API Usage Examples", {
            "pose->SetPose(position, orientation)  // Set 6-DOF pose",
            "pose->SetTrailMode(Pose::TrailMode::kLine)  // Enable line trail",
            "pose->SetAxisColors(red, green, blue)  // Custom axis colors", 
            "pose->SetTrailColor(color)  // Trail color",
            "pose->SetScale(2.0f)  // Scale coordinate frame",
            "pose->SetTransparency(0.7f)  // Semi-transparent rendering"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupPoseScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
/*
 * @file test_line_strip.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Manual test for line strip rendering functionality
 *
 * This test creates a window displaying different line strip types with various rendering options.
 * Run this test to visually verify line strip functionality for paths, trajectories, and boundaries.
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
#include "gldraw/renderable/line_strip.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

class LineStripTestDemo {
public:
    LineStripTestDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager with proper layout settings
        scene_manager_ = std::make_shared<GlSceneManager>("Line Strip Rendering Test");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateTestLineStrips() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // 1. Simple Path - Green solid line
        auto simple_path = std::make_unique<LineStrip>();
        std::vector<glm::vec3> path_points = {
            glm::vec3(-3.0f, 0.0f, 0.0f),
            glm::vec3(-2.0f, 1.0f, 0.0f),
            glm::vec3(-1.0f, 0.5f, 0.0f),
            glm::vec3(0.0f, 1.5f, 0.0f),
            glm::vec3(1.0f, 0.0f, 0.0f),
            glm::vec3(2.0f, 0.5f, 0.0f),
            glm::vec3(3.0f, -0.5f, 0.0f)
        };
        simple_path->SetPoints(path_points);
        simple_path->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        simple_path->SetLineWidth(3.0f);
        scene_manager_->AddOpenGLObject("simple_path", std::move(simple_path));
        
        // 2. Closed Boundary - Red dashed line
        auto boundary = std::make_unique<LineStrip>();
        std::vector<glm::vec3> boundary_points = {
            glm::vec3(-2.0f, 0.0f, 2.0f),
            glm::vec3(-2.0f, 0.0f, 4.0f),
            glm::vec3(0.0f, 0.0f, 4.0f),
            glm::vec3(2.0f, 0.0f, 3.0f),
            glm::vec3(2.0f, 0.0f, 1.0f),
            glm::vec3(0.0f, 0.0f, 2.0f)
        };
        boundary->SetPoints(boundary_points);
        boundary->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        boundary->SetLineWidth(2.0f);
        boundary->SetLineType(LineType::kDashed);
        boundary->SetClosed(true);
        scene_manager_->AddOpenGLObject("boundary", std::move(boundary));
        
        // 3. Trajectory with arrows - Blue line with direction indicators
        auto trajectory = std::make_unique<LineStrip>();
        std::vector<glm::vec3> traj_points;
        for (int i = 0; i <= 40; ++i) {
            float t = i / 40.0f * 2.0f * M_PI;
            float r = 2.0f + 0.5f * sin(3 * t);
            traj_points.push_back(glm::vec3(
                r * cos(t),
                0.5f * sin(2 * t),
                -2.0f + r * sin(t)
            ));
        }
        trajectory->SetPoints(traj_points);
        trajectory->SetColor(glm::vec3(0.0f, 0.5f, 1.0f));
        trajectory->SetLineWidth(2.5f);
        trajectory->SetShowArrows(true, 0.8f);
        trajectory->SetArrowSize(0.15f);
        scene_manager_->AddOpenGLObject("trajectory", std::move(trajectory));
        
        // 4. Path with points - Yellow dotted line with point markers
        auto waypoint_path = std::make_unique<LineStrip>();
        std::vector<glm::vec3> waypoints = {
            glm::vec3(-3.0f, 1.0f, -3.0f),
            glm::vec3(-1.5f, 1.2f, -2.5f),
            glm::vec3(0.0f, 1.5f, -3.0f),
            glm::vec3(1.5f, 1.2f, -3.5f),
            glm::vec3(3.0f, 1.0f, -3.0f)
        };
        waypoint_path->SetPoints(waypoints);
        waypoint_path->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        waypoint_path->SetLineWidth(1.5f);
        waypoint_path->SetLineType(LineType::kDotted);
        waypoint_path->SetShowPoints(true, 8.0f);
        scene_manager_->AddOpenGLObject("waypoint_path", std::move(waypoint_path));
        
        // 5. Multi-colored path - Path with per-vertex colors
        auto colored_path = std::make_unique<LineStrip>();
        std::vector<glm::vec3> colored_points;
        std::vector<glm::vec3> colors;
        for (int i = 0; i <= 20; ++i) {
            float t = i / 20.0f;
            colored_points.push_back(glm::vec3(
                -3.0f + 6.0f * t,
                2.0f + 0.5f * sin(10 * t),
                3.0f
            ));
            // Color gradient from red to blue
            colors.push_back(glm::vec3(1.0f - t, 0.0f, t));
        }
        colored_path->SetPoints(colored_points);
        colored_path->SetColors(colors);
        colored_path->SetLineWidth(4.0f);
        scene_manager_->AddOpenGLObject("colored_path", std::move(colored_path));
        
        // 6. 3D Helix - Purple spiral in 3D space
        auto helix = std::make_unique<LineStrip>();
        std::vector<glm::vec3> helix_points;
        for (int i = 0; i <= 100; ++i) {
            float t = i / 100.0f * 4.0f * M_PI;
            helix_points.push_back(glm::vec3(
                1.5f * cos(t),
                -2.0f + 4.0f * (t / (4.0f * M_PI)),
                1.5f * sin(t)
            ));
        }
        helix->SetPoints(helix_points);
        helix->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
        helix->SetLineWidth(2.0f);
        scene_manager_->AddOpenGLObject("helix", std::move(helix));
        
        std::cout << "\nCreated test scene with:" << std::endl;
        std::cout << "  - Reference grid" << std::endl;
        std::cout << "  - Green simple path (solid)" << std::endl;
        std::cout << "  - Red closed boundary (dashed)" << std::endl;
        std::cout << "  - Blue trajectory with arrows" << std::endl;
        std::cout << "  - Yellow waypoint path (dotted with points)" << std::endl;
        std::cout << "  - Multi-colored gradient path" << std::endl;
        std::cout << "  - Purple 3D helix" << std::endl;
    }
    
    void Run() {
        CreateTestLineStrips();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "R: Reset camera to default position" << std::endl;
        std::cout << "ESC: Exit application" << std::endl;
        
        std::cout << "\n=== Line Strip Features Demonstrated ===" << std::endl;
        std::cout << "- Solid, dashed, and dotted line styles" << std::endl;
        std::cout << "- Open and closed paths" << std::endl;
        std::cout << "- Direction arrows for trajectories" << std::endl;
        std::cout << "- Point markers along paths" << std::endl;
        std::cout << "- Per-vertex coloring for gradients" << std::endl;
        std::cout << "- 3D paths and curves" << std::endl;
        
        // Show viewer
        viewer_.Show();
    }
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Line Strip Rendering Test ===" << std::endl;
    std::cout << "Testing various line strip rendering features for paths and trajectories\n" << std::endl;
    
    try {
        LineStripTestDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
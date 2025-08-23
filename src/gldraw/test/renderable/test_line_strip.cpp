/*
 * @file test_line_strip.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for LineStrip rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/line_strip.hpp"

using namespace quickviz;

void SetupLineStripScene(GlSceneManager* scene_manager) {
    // 1. Straight line - Red
    std::vector<glm::vec3> straight_points = {
        glm::vec3(-4.0f, 0.0f, 0.0f),
        glm::vec3(-2.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(2.0f, 0.0f, 0.0f)
    };
    auto straight_line = std::make_unique<LineStrip>();
    straight_line->SetPoints(straight_points);
    straight_line->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    straight_line->SetLineWidth(3.0f);
    scene_manager->AddOpenGLObject("straight_line", std::move(straight_line));

    // 2. Sine wave - Green
    std::vector<glm::vec3> sine_points;
    for (int i = 0; i <= 50; ++i) {
        float x = -5.0f + (i / 50.0f) * 10.0f;
        float y = 2.0f + std::sin(i / 5.0f) * 1.5f;
        sine_points.push_back(glm::vec3(x, y, 0.0f));
    }
    auto sine_wave = std::make_unique<LineStrip>();
    sine_wave->SetPoints(sine_points);
    sine_wave->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    sine_wave->SetLineWidth(2.0f);
    scene_manager->AddOpenGLObject("sine_wave", std::move(sine_wave));

    // 3. Spiral - Blue
    std::vector<glm::vec3> spiral_points;
    for (int i = 0; i <= 100; ++i) {
        float angle = (i / 100.0f) * 4.0f * M_PI;
        float radius = 0.5f + (i / 100.0f) * 2.0f;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle) - 2.5f;
        spiral_points.push_back(glm::vec3(x, y, 0.0f));
    }
    auto spiral = std::make_unique<LineStrip>();
    spiral->SetPoints(spiral_points);
    spiral->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    spiral->SetLineWidth(4.0f);
    scene_manager->AddOpenGLObject("spiral", std::move(spiral));

    // 4. 3D Helix - Purple
    std::vector<glm::vec3> helix_points;
    for (int i = 0; i <= 80; ++i) {
        float angle = (i / 80.0f) * 6.0f * M_PI;
        float x = 1.5f * std::cos(angle) + 4.0f;
        float y = 1.5f * std::sin(angle);
        float z = (i / 80.0f) * 4.0f - 2.0f;
        helix_points.push_back(glm::vec3(x, y, z));
    }
    auto helix = std::make_unique<LineStrip>();
    helix->SetPoints(helix_points);
    helix->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
    helix->SetLineWidth(3.0f);
    scene_manager->AddOpenGLObject("helix", std::move(helix));

    // 5. Path trajectory - Yellow
    std::vector<glm::vec3> path_points = {
        glm::vec3(-3.0f, -3.0f, 0.0f),
        glm::vec3(-1.0f, -3.5f, 0.0f),
        glm::vec3(1.0f, -3.2f, 0.0f),
        glm::vec3(2.5f, -2.8f, 0.5f),
        glm::vec3(3.0f, -2.0f, 1.0f),
        glm::vec3(2.8f, -1.0f, 1.2f),
        glm::vec3(2.0f, 0.0f, 1.0f),
        glm::vec3(1.0f, 1.5f, 0.5f)
    };
    auto path = std::make_unique<LineStrip>();
    path->SetPoints(path_points);
    path->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    path->SetLineWidth(5.0f);
    scene_manager->AddOpenGLObject("path", std::move(path));

    // 6. Zigzag pattern - Cyan
    std::vector<glm::vec3> zigzag_points;
    for (int i = 0; i <= 20; ++i) {
        float x = -4.0f + (i / 20.0f) * 8.0f;
        float y = ((i % 2) == 0) ? 1.5f : 0.5f;
        zigzag_points.push_back(glm::vec3(x, y, -1.0f));
    }
    auto zigzag = std::make_unique<LineStrip>();
    zigzag->SetPoints(zigzag_points);
    zigzag->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
    zigzag->SetLineWidth(2.5f);
    scene_manager->AddOpenGLObject("zigzag", std::move(zigzag));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "LineStrip Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing line strip rendering for paths, trajectories, and continuous curves");
        
        view.AddHelpSection("LineStrip Features Demonstrated", {
            "- Continuous line segments connecting multiple points",
            "- Different line widths (2.0f to 5.0f pixels)",
            "- Various colors for visual distinction",
            "- 2D and 3D path visualization",
            "- Mathematical curves: sine waves, spirals, helixes",
            "- Practical applications: robot paths, trajectories"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Red straight line: Simple horizontal path",
            "- Green sine wave: Mathematical function visualization",
            "- Blue spiral: 2D expanding circular path",
            "- Purple 3D helix: Corkscrewing 3D trajectory",
            "- Yellow path: Robot/vehicle trajectory example",
            "- Cyan zigzag: Alternating pattern for boundaries"
        });
        
        view.AddHelpSection("Technical Details", {
            "- LineStrip connects points with GL_LINE_STRIP",
            "- Efficient rendering with single draw call per strip",
            "- Variable line widths using OpenGL line width",
            "- Smooth curves with sufficient point density",
            "- 3D depth testing for proper occlusion"
        });
        
        view.AddHelpSection("Applications", {
            "- Robot path planning and visualization",
            "- Vehicle trajectory display",
            "- Sensor data traces (GPS, IMU)",
            "- Mathematical function plotting",
            "- Boundary and contour visualization",
            "- Time-series data representation"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupLineStripScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
/*
 * @file test_path.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for Path rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/path.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

void SetupPathScene(GlSceneManager* scene_manager) {
    // Add grid for reference
    auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // 1. Simple line segments path - uniform color
    auto path1 = std::make_unique<Path>();
    std::vector<glm::vec3> points1 = {
        glm::vec3(-3.0f, -2.0f, 0.0f),
        glm::vec3(-2.0f, -1.0f, 0.5f),
        glm::vec3(-1.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.5f),
        glm::vec3(1.0f, 0.0f, 0.0f)
    };
    path1->SetPoints(points1);
    path1->SetPathType(Path::PathType::kLineSegments);
    path1->SetLineWidth(3.0f);
    path1->SetColor(glm::vec3(1.0f, 0.2f, 0.2f));  // Bright red
    path1->SetColorMode(Path::ColorMode::kUniform);
    scene_manager->AddOpenGLObject("path_line_segments", std::move(path1));
    
    // 2. Smooth curve with gradient colors
    auto path2 = std::make_unique<Path>();
    std::vector<glm::vec3> points2 = {
        glm::vec3(2.0f, -3.0f, 0.0f),
        glm::vec3(3.0f, -2.0f, 1.0f),
        glm::vec3(4.0f, -1.0f, 1.5f),
        glm::vec3(4.5f, 0.0f, 1.0f),
        glm::vec3(4.0f, 1.0f, 0.5f),
        glm::vec3(3.0f, 2.0f, 0.0f)
    };
    path2->SetPoints(points2);
    path2->SetPathType(Path::PathType::kSmoothCurve);
    path2->SetSubdivisions(30);
    path2->SetLineWidth(4.0f);
    path2->SetColorMode(Path::ColorMode::kGradient);
    path2->SetColorGradient(glm::vec3(0.0f, 1.0f, 0.0f),   // Green start
                            glm::vec3(0.0f, 0.0f, 1.0f));   // Blue end
    scene_manager->AddOpenGLObject("path_smooth", std::move(path2));
    
    // 3. Spline curve with velocity encoding
    auto path3 = std::make_unique<Path>();
    std::vector<glm::vec3> points3 = {
        glm::vec3(-4.0f, 1.0f, 0.0f),
        glm::vec3(-3.0f, 2.5f, 0.5f),
        glm::vec3(-1.5f, 3.0f, 1.0f),
        glm::vec3(0.0f, 2.5f, 1.2f),
        glm::vec3(1.5f, 2.0f, 0.8f),
        glm::vec3(2.5f, 1.0f, 0.3f)
    };
    std::vector<float> velocities = {0.5f, 1.2f, 2.0f, 1.8f, 1.0f, 0.3f};
    path3->SetPoints(points3);
    path3->SetPathType(Path::PathType::kSpline);
    path3->SetSubdivisions(25);
    path3->SetLineWidth(5.0f);
    path3->SetColorMode(Path::ColorMode::kVelocity);
    path3->SetScalarValues(velocities);
    path3->SetColorRange(glm::vec2(0.0f, 2.5f));  // Velocity range
    path3->SetTension(0.7f);
    scene_manager->AddOpenGLObject("path_spline", std::move(path3));
    
    // 4. Path with arrows - endpoints mode
    auto path4 = std::make_unique<Path>();
    std::vector<glm::vec3> points4 = {
        glm::vec3(-2.0f, -4.0f, 0.5f),
        glm::vec3(0.0f, -3.5f, 1.0f),
        glm::vec3(2.0f, -4.0f, 0.5f),
        glm::vec3(3.0f, -3.0f, 0.0f)
    };
    path4->SetPoints(points4);
    path4->SetPathType(Path::PathType::kSmoothCurve);
    path4->SetSubdivisions(20);
    path4->SetLineWidth(3.5f);
    path4->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange
    path4->SetArrowMode(Path::ArrowMode::kEndpoints);
    path4->SetArrowSize(0.3f);
    path4->SetArrowColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Yellow arrows
    scene_manager->AddOpenGLObject("path_arrows_endpoints", std::move(path4));
    
    // 5. Bezier curve with regular arrows
    auto path5 = std::make_unique<Path>();
    std::vector<glm::vec3> points5 = {
        glm::vec3(-4.0f, -2.0f, 2.0f),
        glm::vec3(-2.0f, 0.0f, 2.5f),   // Control point
        glm::vec3(0.0f, -1.0f, 2.0f)    // End point
    };
    path5->SetPoints(points5);
    path5->SetPathType(Path::PathType::kBezierCurve);
    path5->SetSubdivisions(40);
    path5->SetLineWidth(4.5f);
    path5->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));  // Magenta
    path5->SetArrowMode(Path::ArrowMode::kRegular);
    path5->SetArrowSpacing(0.8f);
    path5->SetArrowSize(0.25f);
    path5->SetArrowColor(glm::vec3(1.0f, 0.8f, 0.0f));  // Gold arrows
    scene_manager->AddOpenGLObject("path_bezier", std::move(path5));
    
    // 6. Animated path - partial progress
    auto path6 = std::make_unique<Path>();
    std::vector<glm::vec3> points6;
    // Create a spiral path
    for (int i = 0; i <= 20; ++i) {
        float angle = i * 0.6f;
        float radius = 1.5f - i * 0.05f;
        glm::vec3 point = glm::vec3(radius * cos(angle) + 1.0f, 
                                    radius * sin(angle) - 1.0f, 
                                    i * 0.1f + 0.5f);
        points6.push_back(point);
    }
    path6->SetPoints(points6);
    path6->SetPathType(Path::PathType::kSmoothCurve);
    path6->SetSubdivisions(15);
    path6->SetLineWidth(6.0f);
    path6->SetColorMode(Path::ColorMode::kGradient);
    path6->SetColorGradient(glm::vec3(0.0f, 1.0f, 1.0f),   // Cyan start
                            glm::vec3(1.0f, 0.0f, 1.0f));   // Magenta end
    path6->SetAnimationProgress(0.7f);  // Show only 70% of the path
    path6->SetGlowEffect(true, 1.2f);   // Add glow effect
    scene_manager->AddOpenGLObject("path_animated", std::move(path6));
    
    // 7. Custom colored path segments
    auto path7 = std::make_unique<Path>();
    std::vector<glm::vec3> points7 = {
        glm::vec3(3.0f, 3.0f, 0.0f),
        glm::vec3(4.0f, 4.0f, 0.5f),
        glm::vec3(5.0f, 3.5f, 1.0f),
        glm::vec3(5.5f, 2.5f, 1.2f),
        glm::vec3(5.0f, 1.5f, 0.8f)
    };
    std::vector<glm::vec3> colors7 = {
        glm::vec3(1.0f, 0.0f, 0.0f),  // Red
        glm::vec3(1.0f, 1.0f, 0.0f),  // Yellow
        glm::vec3(0.0f, 1.0f, 0.0f),  // Green
        glm::vec3(0.0f, 1.0f, 1.0f),  // Cyan
        glm::vec3(0.0f, 0.0f, 1.0f)   // Blue
    };
    path7->SetPoints(points7);
    path7->SetPathType(Path::PathType::kLineSegments);
    path7->SetLineWidth(7.0f);
    path7->SetColorMode(Path::ColorMode::kCustom);
    path7->SetColors(colors7);
    path7->SetArrowMode(Path::ArrowMode::kAll);
    path7->SetArrowSize(0.2f);
    path7->SetArrowColor(glm::vec3(0.8f, 0.8f, 0.8f));  // Light gray arrows
    scene_manager->AddOpenGLObject("path_custom_colors", std::move(path7));
    
    // 8. Semi-transparent path
    auto path8 = std::make_unique<Path>();
    std::vector<glm::vec3> points8 = {
        glm::vec3(-1.0f, 4.0f, 1.5f),
        glm::vec3(1.0f, 4.5f, 2.0f),
        glm::vec3(3.0f, 4.0f, 1.5f),
        glm::vec3(4.0f, 3.0f, 1.0f)
    };
    path8->SetPoints(points8);
    path8->SetPathType(Path::PathType::kSmoothCurve);
    path8->SetSubdivisions(25);
    path8->SetLineWidth(8.0f);
    path8->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White
    path8->SetTransparency(0.4f);  // Semi-transparent
    path8->SetGlowEffect(true, 0.8f);
    scene_manager->AddOpenGLObject("path_transparent", std::move(path8));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Path Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing path rendering for trajectory and motion planning visualization");
        
        view.AddHelpSection("Path Features Demonstrated", {
            "- Multiple path types: line segments, smooth curves, Bezier, splines",
            "- Various color modes: uniform, gradient, velocity-encoded, custom",
            "- Directional arrows: endpoints, regular spacing, all points",
            "- Animation effects: partial path tracing, glow effects",
            "- Transparency and visual enhancements",
            "- Configurable line widths, arrow sizes, and subdivisions"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Red line segments: Basic waypoint-to-waypoint path",
            "- Green-blue smooth curve: Interpolated trajectory with gradient colors",
            "- Velocity-encoded spline: Color shows speed (blue=slow, red=fast)", 
            "- Orange path with endpoint arrows: Motion direction indicators",
            "- Magenta Bezier curve: Smooth curve with regular arrow spacing",
            "- Cyan-magenta animated spiral: Partial path with glow effect (70% shown)",
            "- Rainbow custom colors: Per-segment color specification",
            "- White transparent path: Semi-transparent elevated trajectory"
        });
        
        view.AddHelpSection("Robotics Applications", {
            "- Robot trajectory visualization and path planning",
            "- Motion planning algorithm results display",
            "- Velocity and acceleration profile visualization",
            "- Multi-waypoint navigation path display",
            "- Obstacle avoidance path visualization",
            "- Formation control and coordination paths",
            "- SLAM trajectory and exploration paths"
        });
        
        view.AddHelpSection("Path Type Details", {
            "- kLineSegments: Direct lines between control points",
            "- kSmoothCurve: Interpolated smooth path through all points",
            "- kBezierCurve: Bezier curve with control points for smooth curves",
            "- kSpline: Catmull-Rom spline for natural motion curves",
            "- Configurable subdivisions for smoothness vs. performance",
            "- Real-time path updates and modifications"
        });
        
        view.AddHelpSection("Color Encoding Modes", {
            "- kUniform: Single color for entire path",
            "- kGradient: Smooth color transition along path length",
            "- kVelocity: Color encodes speed/velocity values",
            "- kTime: Color encodes time parameter along path",
            "- kCost: Color encodes cost/weight values for planning",
            "- kCustom: User-specified colors per path segment"
        });
        
        view.AddHelpSection("API Usage Examples", {
            "path->SetPoints(waypoints)  // Define path control points",
            "path->SetPathType(Path::PathType::kSpline)  // Smooth spline",
            "path->SetColorMode(Path::ColorMode::kVelocity)  // Speed encoding",
            "path->SetArrowMode(Path::ArrowMode::kRegular)  // Direction arrows",
            "path->SetAnimationProgress(0.7f)  // Show 70% of path",
            "path->SetGlowEffect(true, 1.0f)  // Add glow visualization"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupPathScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
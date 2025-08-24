/*
 * @file test_plane.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for Plane rendering functionality
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
#include "gldraw/renderable/plane.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

void SetupPlaneScene(GlSceneManager* scene_manager) {
    // Add grid for reference
    auto grid = std::make_unique<Grid>(15.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // Add coordinate frame for reference
    auto frame = std::make_unique<CoordinateFrame>(2.0f);
    scene_manager->AddOpenGLObject("frame", std::move(frame));
    
    // 1. Horizontal plane (ground) - solid green at Z=1
    auto plane1 = std::make_unique<Plane>();
    plane1->SetCenter(glm::vec3(-3.0f, 3.0f, 1.0f));
    plane1->SetNormal(glm::vec3(0.0f, 0.0f, 1.0f));  // Z-up
    plane1->SetSize(glm::vec2(1.8f, 1.8f));  // Slightly smaller
    plane1->SetColor(glm::vec3(0.2f, 0.8f, 0.2f));  // Green
    plane1->SetRenderMode(Plane::RenderMode::kSolid);
    plane1->SetOpacity(0.8f);
    scene_manager->AddOpenGLObject("plane_horizontal", std::move(plane1));
    
    // 2. Vertical plane (wall) - wireframe blue
    auto plane2 = std::make_unique<Plane>();
    plane2->SetCenter(glm::vec3(5.0f, 0.0f, 2.0f));
    plane2->SetNormal(glm::vec3(-1.0f, 0.0f, 0.0f));  // Facing -X
    plane2->SetSize(glm::vec2(3.0f, 3.0f));
    plane2->SetRenderMode(Plane::RenderMode::kWireframe);
    plane2->SetWireframeColor(glm::vec3(0.2f, 0.6f, 1.0f));  // Blue
    plane2->SetWireframeWidth(2.0f);
    plane2->SetShowGrid(true);
    plane2->SetGridResolution(6, 6);
    scene_manager->AddOpenGLObject("plane_wall", std::move(plane2));
    
    // 3. Tilted plane - transparent orange with grid
    auto plane3 = std::make_unique<Plane>();
    plane3->SetCenter(glm::vec3(0.0f, 4.0f, 2.5f));
    glm::vec3 normal = glm::normalize(glm::vec3(0.0f, -0.5f, 1.0f));
    plane3->SetNormal(normal);
    plane3->SetSize(glm::vec2(2.5f, 2.5f));
    plane3->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange
    plane3->SetRenderMode(Plane::RenderMode::kTransparent);
    plane3->SetOpacity(0.5f);
    plane3->SetShowGrid(true);
    plane3->SetGridColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Yellow grid
    plane3->SetGridResolution(5, 5);
    scene_manager->AddOpenGLObject("plane_tilted", std::move(plane3));
    
    // 4. Plane defined by corners - magenta vertical wall
    auto plane4 = std::make_unique<Plane>(
        glm::vec3(-5.0f, -3.0f, 0.0f),  // Corner 1
        glm::vec3(-3.0f, -3.0f, 0.0f),  // Corner 2
        glm::vec3(-3.0f, -3.0f, 3.0f),  // Corner 3
        glm::vec3(-5.0f, -3.0f, 3.0f)   // Corner 4
    );
    plane4->SetColor(glm::vec3(1.0f, 0.0f, 1.0f));  // Magenta
    plane4->SetRenderMode(Plane::RenderMode::kSolid);
    plane4->SetOpacity(0.9f);
    scene_manager->AddOpenGLObject("plane_corners", std::move(plane4));
    
    // 5. High-resolution plane with texture coordinates - cyan
    auto plane5 = std::make_unique<Plane>();
    plane5->SetCenter(glm::vec3(3.0f, 3.0f, 3.5f));
    plane5->SetNormal(glm::vec3(0.0f, -0.7f, 0.7f));  // Angled down
    plane5->SetSize(glm::vec2(2.0f, 2.0f));
    plane5->SetColor(glm::vec3(0.0f, 0.8f, 0.8f));  // Cyan
    plane5->SetRenderMode(Plane::RenderMode::kSolid);
    plane5->SetGridResolution(10, 10);  // Higher resolution mesh
    plane5->SetTextureCoordinates(true);  // Enable texture coords
    plane5->SetOpacity(0.85f);
    scene_manager->AddOpenGLObject("plane_highres", std::move(plane5));
    
    // 6. Plane with normal visualization - red
    auto plane6 = std::make_unique<Plane>();
    plane6->SetCenter(glm::vec3(-4.0f, 0.0f, 2.5f));
    plane6->SetNormal(glm::vec3(0.7f, 0.0f, 0.7f));  // Diagonal normal
    plane6->SetSize(glm::vec2(1.5f, 1.5f));
    plane6->SetColor(glm::vec3(0.8f, 0.2f, 0.2f));  // Red
    plane6->SetRenderMode(Plane::RenderMode::kTransparent);
    plane6->SetOpacity(0.6f);
    plane6->SetShowNormal(true, 2.0f);  // Show normal with length 2.0
    plane6->SetNormalColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Yellow normal
    scene_manager->AddOpenGLObject("plane_normal", std::move(plane6));
    
    // 7. Point cloud visualization mode - white dots
    auto plane7 = std::make_unique<Plane>();
    plane7->SetCenter(glm::vec3(3.0f, -3.0f, 1.5f));
    plane7->SetNormal(glm::vec3(0.0f, 0.0f, 1.0f));  // Horizontal
    plane7->SetSize(glm::vec2(2.0f, 2.0f));
    plane7->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White
    plane7->SetRenderMode(Plane::RenderMode::kPoints);
    plane7->SetGridResolution(8, 8);  // Points at grid intersections
    scene_manager->AddOpenGLObject("plane_points", std::move(plane7));
    
    // 8. Transformed plane using matrix - purple (more visible position)
    auto plane8 = std::make_unique<Plane>();
    plane8->SetCenter(glm::vec3(0.0f, 0.0f, 0.0f));  // Local origin
    plane8->SetNormal(glm::vec3(0.0f, 0.0f, 1.0f));  // Local Z-up
    plane8->SetSize(glm::vec2(2.0f, 2.5f));
    plane8->SetColor(glm::vec3(0.6f, 0.2f, 0.8f));  // Purple
    plane8->SetRenderMode(Plane::RenderMode::kSolid);
    plane8->SetOpacity(0.75f);
    // Apply transformation: rotate around Y and translate to a more visible position
    glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(-6.0f, -2.0f, 1.5f));
    transform = glm::rotate(transform, glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    plane8->SetTransform(transform);
    scene_manager->AddOpenGLObject("plane_transformed", std::move(plane8));
    
    // 9. Wireframe outline only (no grid) - bright yellow
    auto plane9 = std::make_unique<Plane>();
    plane9->SetCenter(glm::vec3(-2.0f, -4.0f, 3.5f));
    plane9->SetNormal(glm::vec3(0.0f, 0.5f, 0.5f));  // Angled
    plane9->SetSize(glm::vec2(2.0f, 2.0f));
    plane9->SetRenderMode(Plane::RenderMode::kWireframe);
    plane9->SetWireframeColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Bright yellow
    plane9->SetWireframeWidth(3.0f);
    plane9->SetShowGrid(false);  // Only outline, no internal grid
    scene_manager->AddOpenGLObject("plane_outline", std::move(plane9));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Plane Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing plane rendering with various orientations and visualization modes");
        
        view.AddHelpSection("Plane Features Demonstrated", {
            "- Arbitrary plane orientation via normal vectors",
            "- Multiple construction methods: center+normal, corners, transform",
            "- Render modes: solid, wireframe, transparent, points",
            "- Configurable grid resolution and visibility",
            "- Normal vector visualization for orientation",
            "- Texture coordinate generation support",
            "- Opacity and transparency control"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Green horizontal: Ground plane at (-3,3,1) facing up with semi-transparency",
            "- Blue wireframe wall: Vertical plane at (5,0,2) with 6x6 grid lines",
            "- Orange tilted: Transparent angled plane at (0,4,2.5) with yellow grid",
            "- Magenta wall: Vertical plane at left side (-5,-3,1.5) defined by 4 corners",
            "- Cyan angled: High-res plane at (3,3,3.5) with texture coordinates",
            "- Red with normal: Diagonal plane at (-4,0,2.5) showing yellow normal vector",
            "- White points: Point cloud mode at (3,-3,1.5) showing grid intersections",
            "- Purple transformed: Rotated 45° plane at (-6,-2,1.5) with transformation matrix",
            "- Yellow outline: Wireframe at (-2,-4,3.5) with border only, no internal grid"
        });
        
        view.AddHelpSection("Robotics Applications", {
            "- Ground plane and terrain representation",
            "- Wall and obstacle surfaces",
            "- Robot workspace boundaries",
            "- Sensor detection planes",
            "- Cross-section visualization",
            "- Camera image planes",
            "- Clipping and slicing planes",
            "- Surface fitting and RANSAC results"
        });
        
        view.AddHelpSection("Construction Methods", {
            "- SetCenter() + SetNormal(): Define by point and normal",
            "- SetFromCorners(): Define by 4 corner points",
            "- SetFromPointAndNormal(): Combined setter",
            "- SetTransform(): Apply 4x4 transformation matrix",
            "- GetPlaneEquation(): Returns ax+by+cz+d=0 coefficients"
        });
        
        view.AddHelpSection("Visualization Options", {
            "- kSolid: Filled surface with lighting",
            "- kWireframe: Grid lines or outline only",
            "- kTransparent: See-through surface with alpha",
            "- kPoints: Vertex points at grid intersections",
            "- SetShowGrid(): Toggle internal grid lines",
            "- SetShowNormal(): Display normal vector arrow",
            "- SetGridResolution(): Mesh subdivision level"
        });
        
        view.AddHelpSection("API Usage Examples", {
            "plane->SetCenter(center)  // Set plane center point",
            "plane->SetNormal(normal)  // Set plane normal vector",
            "plane->SetSize(vec2(w,h))  // Set plane dimensions",
            "plane->SetRenderMode(Plane::RenderMode::kTransparent)",
            "plane->SetShowGrid(true)  // Enable grid lines",
            "plane->SetOpacity(0.5f)  // Semi-transparent"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupPlaneScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
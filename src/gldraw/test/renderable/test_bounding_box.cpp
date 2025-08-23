/*
 * @file test_bounding_box.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for BoundingBox rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/bounding_box.hpp"

using namespace quickviz;

void SetupBoundingBoxScene(GlSceneManager* scene_manager) {
    // 1. Medium red box - left front
    auto box1 = std::make_unique<BoundingBox>(glm::vec3(-3.0f, -1.0f, 2.0f), glm::vec3(-1.5f, 1.0f, 3.5f));
    box1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));  // Pure red
    box1->SetRenderMode(BoundingBox::RenderMode::kSolid);
    scene_manager->AddOpenGLObject("box1", std::move(box1));
    
    // 2. Medium green box - right front (moved away from overlapping area)
    auto box2 = std::make_unique<BoundingBox>(glm::vec3(4.0f, -1.0f, -2.0f), glm::vec3(5.5f, 1.0f, -3.5f));
    box2->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));  // Pure green
    box2->SetRenderMode(BoundingBox::RenderMode::kSolid);
    scene_manager->AddOpenGLObject("box2", std::move(box2));
    
    // 3. Small orange box - center
    auto box3 = std::make_unique<BoundingBox>(glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.5f, 0.5f, 1.0f));
    box3->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Bright orange
    box3->SetRenderMode(BoundingBox::RenderMode::kSolid);
    scene_manager->AddOpenGLObject("box3", std::move(box3));
    
    // 4. Yellow wireframe box - left back
    auto box4 = std::make_unique<BoundingBox>(glm::vec3(-3.0f, -1.0f, -3.5f), glm::vec3(-1.5f, 1.0f, -2.0f));
    box4->SetRenderMode(BoundingBox::RenderMode::kWireframe);
    box4->SetEdgeColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Bright yellow edges
    box4->SetEdgeWidth(4.0f);  // Thick edges
    box4->SetShowEdges(true);
    box4->SetShowFaces(false);
    scene_manager->AddOpenGLObject("box4", std::move(box4));
    
    // 5. Magenta transparent box - separate position to show transparency effect
    auto box5 = std::make_unique<BoundingBox>(glm::vec3(1.5f, -1.0f, -1.0f), glm::vec3(3.0f, 1.0f, 0.5f));
    box5->SetColor(glm::vec3(1.0f, 0.0f, 1.0f));  // Pure magenta
    box5->SetRenderMode(BoundingBox::RenderMode::kTransparent);
    box5->SetOpacity(0.3f);  // More visible transparent for better demo
    box5->SetShowFaces(true);  // Show transparent faces
    box5->SetShowEdges(true);  // Show edges for structure
    box5->SetEdgeColor(glm::vec3(1.0f, 0.5f, 1.0f));  // Lighter magenta edges
    box5->SetEdgeWidth(1.5f);  // Thin edges
    scene_manager->AddOpenGLObject("box5", std::move(box5));
    
    // 6. Cyan wireframe with corner points - elevated center
    auto box6 = std::make_unique<BoundingBox>(glm::vec3(-0.75f, 2.0f, -0.75f), glm::vec3(0.75f, 3.0f, 0.75f));
    box6->SetRenderMode(BoundingBox::RenderMode::kWireframe);
    box6->SetEdgeColor(glm::vec3(0.0f, 1.0f, 1.0f));  // Bright cyan edges
    box6->SetEdgeWidth(3.0f);  // Thick edges
    box6->SetShowEdges(true);
    box6->SetShowFaces(false);
    box6->SetShowCornerPoints(true, 8.0f);  // Show corner points
    scene_manager->AddOpenGLObject("box6", std::move(box6));
    
    // 7. Rotated blue solid box - 45° around Y axis
    auto box7 = std::make_unique<BoundingBox>(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f));
    box7->SetColor(glm::vec3(0.0f, 0.5f, 1.0f));  // Bright blue
    box7->SetRenderMode(BoundingBox::RenderMode::kSolid);
    // Rotate 45° around Y axis and translate to position
    glm::mat4 transform7 = glm::translate(glm::mat4(1.0f), glm::vec3(4.5f, 0.0f, 0.0f));
    transform7 = transform7 * glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    box7->SetTransform(transform7);
    scene_manager->AddOpenGLObject("box7", std::move(box7));
    
    // 8. Rotated white wireframe - 30° around X and Z axes
    auto box8 = std::make_unique<BoundingBox>(glm::vec3(-0.75f, -0.75f, -0.75f), glm::vec3(0.75f, 0.75f, 0.75f));
    box8->SetRenderMode(BoundingBox::RenderMode::kWireframe);
    box8->SetEdgeColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White edges
    box8->SetEdgeWidth(2.5f);
    box8->SetShowEdges(true);
    box8->SetShowFaces(false);
    // Combine rotations: 30° around X, then 30° around Z, then translate
    glm::mat4 transform8 = glm::translate(glm::mat4(1.0f), glm::vec3(-4.5f, 0.0f, 0.0f));
    transform8 = transform8 * glm::rotate(glm::mat4(1.0f), glm::radians(30.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    transform8 = transform8 * glm::rotate(glm::mat4(1.0f), glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    box8->SetTransform(transform8);
    scene_manager->AddOpenGLObject("box8", std::move(box8));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "BoundingBox Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing bounding box rendering for object bounds and region visualization");
        
        view.AddHelpSection("BoundingBox Features Demonstrated", {
            "- Axis-aligned bounding boxes (AABB)",
            "- Rotated bounding boxes using transform matrices",
            "- Different sizes and positions",
            "- Various colors for visual distinction", 
            "- Enhanced wireframe rendering with thick edges",
            "- Corner point visualization for better structure",
            "- Transparency for overlapping regions",
            "- Combined rotation transformations (multi-axis)"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Red solid box: Left front position (-3,-1,2) to (-1.5,1,3.5)",
            "- Green solid box: Far right position (4,-1,2) to (5.5,1,3.5) - now clearly visible",
            "- Orange solid box: Small center box (-0.5,-0.5,0) to (0.5,0.5,1)",
            "- Yellow wireframe: Thick edges at left back (-3,-1,-3.5) to (-1.5,1,-2)",
            "- Magenta transparent: Independent position (1.5,-1,-1) to (3,1,0.5)",
            "- Cyan wireframe: Elevated with corner points (-0.75,2,-0.75) to (0.75,3,0.75)",
            "- Blue rotated solid: 45° Y-axis rotation at (4.5,0,0)",
            "- White rotated wireframe: 30° X + 30° Z rotation at (-4.5,0,0)"
        });
        
        view.AddHelpSection("Use Cases", {
            "- Object collision detection boundaries",
            "- Spatial partitioning visualization", 
            "- Region-of-interest highlighting",
            "- Robotics workspace definition",
            "- Game development collision boxes",
            "- 3D model bounds display"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupBoundingBoxScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
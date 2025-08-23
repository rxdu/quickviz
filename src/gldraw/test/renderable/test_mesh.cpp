/*
 * @file test_mesh.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-01-22
 * @brief Test for Mesh rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/mesh.hpp"

using namespace quickviz;

// Helper function to create triangle mesh
std::unique_ptr<Mesh> CreateTriangleMesh() {
    auto mesh = std::make_unique<Mesh>();
    
    std::vector<glm::vec3> vertices = {
        glm::vec3(-3.0f, 0.5f, 0.0f),   // Top
        glm::vec3(-3.5f, -0.5f, 0.0f), // Bottom left  
        glm::vec3(-2.5f, -0.5f, 0.0f)  // Bottom right
    };
    
    std::vector<uint32_t> indices = {0, 1, 2};
    
    mesh->SetVertices(vertices);
    mesh->SetIndices(indices);
    
    return mesh;
}

// Helper function to create cube mesh
std::unique_ptr<Mesh> CreateCubeMesh() {
    auto mesh = std::make_unique<Mesh>();
    
    std::vector<glm::vec3> vertices = {
        // Front face
        glm::vec3(-2.0f, -0.5f,  0.5f), glm::vec3(-1.0f, -0.5f,  0.5f),
        glm::vec3(-1.0f,  0.5f,  0.5f), glm::vec3(-2.0f,  0.5f,  0.5f),
        // Back face  
        glm::vec3(-2.0f, -0.5f, -0.5f), glm::vec3(-1.0f, -0.5f, -0.5f),
        glm::vec3(-1.0f,  0.5f, -0.5f), glm::vec3(-2.0f,  0.5f, -0.5f)
    };
    
    std::vector<uint32_t> indices = {
        0, 1, 2, 2, 3, 0,  // Front
        4, 6, 5, 6, 4, 7,  // Back  
        4, 0, 3, 3, 7, 4,  // Left
        1, 5, 6, 6, 2, 1,  // Right
        3, 2, 6, 6, 7, 3,  // Top
        4, 5, 1, 1, 0, 4   // Bottom
    };
    
    mesh->SetVertices(vertices);
    mesh->SetIndices(indices);
    
    return mesh;
}

// Helper function to create sphere mesh
std::unique_ptr<Mesh> CreateSphereMesh(float radius, int slices, int stacks) {
    auto mesh = std::make_unique<Mesh>();
    
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices;
    
    // Generate sphere vertices
    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * float(i) / float(stacks);
        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * M_PI * float(j) / float(slices);
            
            float x = radius * std::sin(phi) * std::cos(theta) + 1.0f;
            float y = radius * std::cos(phi);
            float z = radius * std::sin(phi) * std::sin(theta);
            
            vertices.push_back(glm::vec3(x, y, z));
        }
    }
    
    // Generate sphere indices
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int first = (i * (slices + 1)) + j;
            int second = first + slices + 1;
            
            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(first + 1);
            
            indices.push_back(second);
            indices.push_back(second + 1);
            indices.push_back(first + 1);
        }
    }
    
    mesh->SetVertices(vertices);
    mesh->SetIndices(indices);
    
    return mesh;
}

// Helper function to create plane mesh
std::unique_ptr<Mesh> CreatePlaneMesh(float width, float height, int segments_x, int segments_y) {
    auto mesh = std::make_unique<Mesh>();
    
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices;
    
    float step_x = width / float(segments_x);
    float step_y = height / float(segments_y);
    
    // Generate plane vertices
    for (int i = 0; i <= segments_y; ++i) {
        for (int j = 0; j <= segments_x; ++j) {
            float x = -width / 2.0f + float(j) * step_x + 3.0f;
            float y = -height / 2.0f + float(i) * step_y;
            float z = -2.0f;
            
            vertices.push_back(glm::vec3(x, y, z));
        }
    }
    
    // Generate plane indices
    for (int i = 0; i < segments_y; ++i) {
        for (int j = 0; j < segments_x; ++j) {
            int first = i * (segments_x + 1) + j;
            int second = (i + 1) * (segments_x + 1) + j;
            
            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(first + 1);
            
            indices.push_back(second);
            indices.push_back(second + 1);
            indices.push_back(first + 1);
        }
    }
    
    mesh->SetVertices(vertices);
    mesh->SetIndices(indices);
    
    return mesh;
}

void SetupMeshScene(GlSceneManager* scene_manager) {
    // 1. Simple Triangle - Red
    auto triangle = CreateTriangleMesh();
    triangle->SetColor(glm::vec3(0.9f, 0.1f, 0.1f));
    scene_manager->AddOpenGLObject("triangle", std::move(triangle));
    
    // 2. Cube with wireframe - Green with white wireframe
    auto cube = CreateCubeMesh();
    cube->SetColor(glm::vec3(0.1f, 0.8f, 0.1f));
    cube->SetWireframeMode(true);
    cube->SetWireframeColor(glm::vec3(1.0f, 1.0f, 1.0f));
    scene_manager->AddOpenGLObject("cube", std::move(cube));
    
    // 3. Sphere with transparency - Blue
    auto sphere = CreateSphereMesh(0.8f, 20, 12);
    sphere->SetColor(glm::vec3(0.1f, 0.3f, 0.9f));
    sphere->SetTransparency(0.7f);
    scene_manager->AddOpenGLObject("sphere", std::move(sphere));
    
    // 4. Plane with normals - Yellow
    auto plane = CreatePlaneMesh(2.0f, 2.0f, 4, 4);
    plane->SetColor(glm::vec3(0.9f, 0.9f, 0.2f));
    plane->SetShowNormals(true, 0.3f);
    plane->SetNormalColor(glm::vec3(0.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("plane", std::move(plane));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Mesh Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing mesh rendering with various shapes, materials, and rendering modes");
        
        view.AddHelpSection("Mesh Features Demonstrated", {
            "- Custom mesh creation with vertices and indices",
            "- Different primitive shapes: triangle, cube, sphere, plane",
            "- Wireframe rendering mode",
            "- Transparency effects (alpha blending)",
            "- Surface normal visualization",
            "- Custom colors and materials"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Red Triangle: Simple 3-vertex mesh at (-3, 0, 0)",
            "- Green Cube: 8-vertex box with white wireframe at (-1.5, 0, 0)",
            "- Blue Sphere: Procedurally generated at (1, 0, 0) with transparency",
            "- Yellow Plane: Grid mesh at (3, 0, -2) showing surface normals",
            "- Reference grid and coordinate frame for spatial context"
        });
        
        view.AddHelpSection("Rendering Modes", {
            "- Solid fill: Triangle and plane base rendering",
            "- Wireframe overlay: Cube shows both solid and wireframe",
            "- Transparency: Sphere demonstrates alpha blending",
            "- Normal visualization: Plane shows surface normal vectors",
            "- Color coding: Each shape has distinct material color"
        });
        
        view.AddHelpSection("Technical Details", {
            "- Triangle: 3 vertices, 1 triangle",
            "- Cube: 8 vertices, 12 triangles (6 faces)",
            "- Sphere: ~260 vertices, ~480 triangles (20x12 resolution)",
            "- Plane: 25 vertices, 32 triangles (4x4 grid)",
            "- All meshes use indexed rendering for efficiency"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupMeshScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
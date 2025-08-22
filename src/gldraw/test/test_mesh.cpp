/*
 * @file test_mesh.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-01-22
 * @brief Manual test for mesh rendering functionality - demonstrates various mesh features
 *
 * This test creates a window displaying different mesh types with various rendering options.
 * Run this test to visually verify mesh functionality and see what to expect from the Mesh class.
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
#include "gldraw/renderable/mesh.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

class MeshTestDemo {
public:
    MeshTestDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager with proper layout settings
        scene_manager_ = std::make_shared<GlSceneManager>("Mesh Rendering Test");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateTestMeshes() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // 1. Simple Triangle - Red
        auto triangle = CreateTriangleMesh();
        triangle->SetColor(glm::vec3(0.9f, 0.1f, 0.1f));
        scene_manager_->AddOpenGLObject("triangle", std::move(triangle));
        
        // 2. Cube with wireframe - Green with white wireframe
        auto cube = CreateCubeMesh();
        cube->SetColor(glm::vec3(0.1f, 0.8f, 0.1f));
        cube->SetWireframeMode(true);
        cube->SetWireframeColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White wireframe for visibility
        scene_manager_->AddOpenGLObject("cube", std::move(cube));
        
        // 3. Sphere with transparency - Blue
        auto sphere = CreateSphereMesh(0.8f, 20, 12);
        sphere->SetColor(glm::vec3(0.1f, 0.3f, 0.9f));
        sphere->SetTransparency(0.7f);
        scene_manager_->AddOpenGLObject("sphere", std::move(sphere));
        
        // 4. Torus - Purple/Magenta
        auto torus = CreateTorusMesh(0.6f, 0.3f, 20, 16);
        torus->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
        scene_manager_->AddOpenGLObject("torus", std::move(torus));
        
        // 5. Plane with normals - Yellow
        auto plane = CreatePlaneMesh(2.0f, 2.0f, 4, 4);
        plane->SetColor(glm::vec3(0.9f, 0.9f, 0.2f));
        plane->SetShowNormals(true, 0.3f);
        plane->SetNormalColor(glm::vec3(0.0f, 1.0f, 0.0f));
        scene_manager_->AddOpenGLObject("plane", std::move(plane));
        
        std::cout << "\nCreated test scene with:" << std::endl;
        std::cout << "  - Reference grid" << std::endl;
        std::cout << "  - Red Triangle (simple)" << std::endl;
        std::cout << "  - Green Cube (white wireframe)" << std::endl;
        std::cout << "  - Blue Sphere (transparent)" << std::endl;
        std::cout << "  - Purple Torus (solid)" << std::endl;
        std::cout << "  - Yellow Plane (with normals)" << std::endl;
    }
    
    void Run() {
        CreateTestMeshes();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "Right Mouse: Alternative rotation" << std::endl;
        std::cout << "\nExpected visuals:" << std::endl;
        std::cout << "  - Various colored 3D shapes with different rendering modes" << std::endl;
        std::cout << "  - Green arrows on yellow plane showing surface normals" << std::endl;
        std::cout << "  - Semi-transparent blue sphere" << std::endl;
        std::cout << "  - White wireframe outline on green cube" << std::endl;
        
        viewer_.Show();
    }

private:
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
    
    std::unique_ptr<Mesh> CreateSphereMesh(float radius, int segments, int rings) {
        auto mesh = std::make_unique<Mesh>();
        
        std::vector<glm::vec3> vertices;
        std::vector<uint32_t> indices;
        
        // Generate sphere vertices
        for (int ring = 0; ring <= rings; ++ring) {
            float theta = static_cast<float>(ring) * M_PI / static_cast<float>(rings);
            float sin_theta = std::sin(theta);
            float cos_theta = std::cos(theta);
            
            for (int seg = 0; seg <= segments; ++seg) {
                float phi = static_cast<float>(seg) * 2.0f * M_PI / static_cast<float>(segments);
                float sin_phi = std::sin(phi);
                float cos_phi = std::cos(phi);
                
                float x = radius * sin_theta * cos_phi;
                float y = radius * cos_theta;
                float z = radius * sin_theta * sin_phi;
                
                vertices.emplace_back(x + 1.0f, y, z);  // Offset position
            }
        }
        
        // Generate sphere indices
        for (int ring = 0; ring < rings; ++ring) {
            for (int seg = 0; seg < segments; ++seg) {
                uint32_t current = ring * (segments + 1) + seg;
                uint32_t next = current + segments + 1;
                
                indices.push_back(current);
                indices.push_back(next);
                indices.push_back(current + 1);
                
                indices.push_back(current + 1);
                indices.push_back(next);
                indices.push_back(next + 1);
            }
        }
        
        mesh->SetVertices(vertices);
        mesh->SetIndices(indices);
        
        return mesh;
    }
    
    std::unique_ptr<Mesh> CreateTorusMesh(float major_radius, float minor_radius, 
                                         int major_segments, int minor_segments) {
        auto mesh = std::make_unique<Mesh>();
        
        std::vector<glm::vec3> vertices;
        std::vector<uint32_t> indices;
        
        // Generate torus vertices
        for (int i = 0; i <= major_segments; ++i) {
            float u = static_cast<float>(i) * 2.0f * M_PI / static_cast<float>(major_segments);
            float cos_u = std::cos(u);
            float sin_u = std::sin(u);
            
            for (int j = 0; j <= minor_segments; ++j) {
                float v = static_cast<float>(j) * 2.0f * M_PI / static_cast<float>(minor_segments);
                float cos_v = std::cos(v);
                float sin_v = std::sin(v);
                
                float x = (major_radius + minor_radius * cos_v) * cos_u;
                float y = minor_radius * sin_v;
                float z = (major_radius + minor_radius * cos_v) * sin_u;
                
                vertices.emplace_back(x + 3.0f, y, z);  // Offset position
            }
        }
        
        // Generate torus indices  
        for (int i = 0; i < major_segments; ++i) {
            for (int j = 0; j < minor_segments; ++j) {
                uint32_t current = i * (minor_segments + 1) + j;
                uint32_t next_i = ((i + 1) % major_segments) * (minor_segments + 1) + j;
                uint32_t next_j = i * (minor_segments + 1) + ((j + 1) % minor_segments);
                uint32_t next_both = ((i + 1) % major_segments) * (minor_segments + 1) + 
                                   ((j + 1) % minor_segments);
                
                indices.push_back(current);
                indices.push_back(next_i);
                indices.push_back(next_j);
                
                indices.push_back(next_j);
                indices.push_back(next_i);
                indices.push_back(next_both);
            }
        }
        
        mesh->SetVertices(vertices);
        mesh->SetIndices(indices);
        
        return mesh;
    }
    
    std::unique_ptr<Mesh> CreatePlaneMesh(float width, float height, int width_segs, int height_segs) {
        auto mesh = std::make_unique<Mesh>();
        
        std::vector<glm::vec3> vertices;
        std::vector<uint32_t> indices;
        
        // Generate plane vertices
        for (int i = 0; i <= height_segs; ++i) {
            for (int j = 0; j <= width_segs; ++j) {
                float x = (static_cast<float>(j) / static_cast<float>(width_segs)) * width - width * 0.5f;
                float y = -1.0f;  // Place below other objects
                float z = (static_cast<float>(i) / static_cast<float>(height_segs)) * height - height * 0.5f;
                
                vertices.emplace_back(x, y, z);
            }
        }
        
        // Generate plane indices
        for (int i = 0; i < height_segs; ++i) {
            for (int j = 0; j < width_segs; ++j) {
                uint32_t current = i * (width_segs + 1) + j;
                uint32_t next_i = (i + 1) * (width_segs + 1) + j;
                uint32_t next_j = i * (width_segs + 1) + (j + 1);
                uint32_t next_both = (i + 1) * (width_segs + 1) + (j + 1);
                
                indices.push_back(current);
                indices.push_back(next_i);
                indices.push_back(next_j);
                
                indices.push_back(next_j);
                indices.push_back(next_i);
                indices.push_back(next_both);
            }
        }
        
        mesh->SetVertices(vertices);
        mesh->SetIndices(indices);
        
        return mesh;
    }

    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main() {
    try {
        std::cout << "=== QuickViz Mesh Test Demo ===" << std::endl;
        std::cout << "This demo shows various mesh rendering capabilities:" << std::endl;
        std::cout << "- Basic triangle mesh" << std::endl;
        std::cout << "- Cube with wireframe mode" << std::endl;
        std::cout << "- Semi-transparent sphere" << std::endl;
        std::cout << "- Solid torus" << std::endl;
        std::cout << "- Plane with normal visualization" << std::endl;
        std::cout << std::endl;
        
        MeshTestDemo demo;
        demo.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
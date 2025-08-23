/*
 * @file test_text3d_minimal.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Minimal test for Text3D without complex viewer dependencies
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/renderable/text3d.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

class MinimalText3DTest {
public:
    MinimalText3DTest() : window_(nullptr) {}
    
    ~MinimalText3DTest() {
        if (window_) {
            glfwTerminate();
        }
    }
    
    bool Initialize() {
        // Initialize GLFW
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return false;
        }
        
        // Set OpenGL context hints
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        
        // Create window
        window_ = glfwCreateWindow(800, 600, "Minimal Text3D Test", nullptr, nullptr);
        if (!window_) {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return false;
        }
        
        glfwMakeContextCurrent(window_);
        
        // Initialize GLAD
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            std::cerr << "Failed to initialize GLAD" << std::endl;
            return false;
        }
        
        // Set viewport
        glViewport(0, 0, 800, 600);
        
        // Enable depth testing
        glEnable(GL_DEPTH_TEST);
        
        // Set clear color
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        
        std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
        
        return true;
    }
    
    void Run() {
        if (!Initialize()) {
            return;
        }
        
        // Skip grid to avoid shader conflicts
        // auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
        // grid->AllocateGpuResources();
        
        // Create text object
        auto text = std::make_unique<Text3D>();
        text->SetText("HELLO");
        text->SetPosition(glm::vec3(0.0f, 1.0f, 0.0f));
        text->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));
        text->SetScale(2.0f);
        text->SetBillboardMode(Text3D::BillboardMode::kSphere);
        text->SetAlignment(Text3D::Alignment::kCenter, Text3D::VerticalAlignment::kMiddle);
        
        std::cout << "Allocating Text3D GPU resources..." << std::endl;
        text->AllocateGpuResources();
        std::cout << "Text3D GPU resources allocated successfully!" << std::endl;
        
        // Camera setup
        glm::vec3 camera_pos(5.0f, 3.0f, 5.0f);
        glm::vec3 camera_target(0.0f, 0.0f, 0.0f);
        glm::vec3 camera_up(0.0f, 1.0f, 0.0f);
        
        double last_time = glfwGetTime();
        
        std::cout << "Starting render loop. Press ESC to exit." << std::endl;
        
        while (!glfwWindowShouldClose(window_)) {
            double current_time = glfwGetTime();
            double delta_time = current_time - last_time;
            last_time = current_time;
            
            // Handle input
            if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window_, true);
            }
            
            // Rotate camera around origin
            float angle = static_cast<float>(current_time) * 0.2f;
            float radius = 8.0f;
            camera_pos.x = radius * cos(angle);
            camera_pos.z = radius * sin(angle);
            camera_pos.y = 3.0f;
            
            // Clear buffers
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            // Setup matrices
            glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
            glm::mat4 view = glm::lookAt(camera_pos, camera_target, camera_up);
            glm::mat4 coord_transform = glm::mat4(1.0f);
            
            // Skip grid rendering
            // grid->OnDraw(projection, view, coord_transform);
            
            // Render text
            text->OnDraw(projection, view, coord_transform);
            
            // Swap buffers and poll events
            glfwSwapBuffers(window_);
            glfwPollEvents();
        }
        
        std::cout << "Render loop finished" << std::endl;
    }

private:
    GLFWwindow* window_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Minimal Text3D Test ===\n";
    std::cout << "Testing Text3D bitmap font system without ImGui dependencies\n";
    
    MinimalText3DTest test;
    test.Run();
    
    return 0;
}
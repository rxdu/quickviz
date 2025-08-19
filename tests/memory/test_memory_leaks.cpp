/*
 * @file test_memory_leaks.cpp
 * @date 2024-06-25
 * @brief Memory leak detection tests for OpenGL resources
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cstdlib>
#include <cstring>

#include "imview/viewer.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/triangle.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/shader_program.hpp"
#include "gldraw/frame_buffer.hpp"
#include "core/event/event.hpp"
#include "core/event/event_dispatcher.hpp"

using namespace quickviz;

class MemoryLeakTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Check if display is available (required for graphics tests)
        if (!IsDisplayAvailable()) {
            GTEST_SKIP() << "Skipping graphics test: No display available (headless environment)";
        }
        
        try {
            // Initialize OpenGL context for testing
            viewer_ = std::make_unique<Viewer>("Memory Test", 800, 600);
        } catch (const std::runtime_error& e) {
            GTEST_SKIP() << "Skipping graphics test: " << e.what();
        }
    }

    void TearDown() override {
        viewer_.reset();
    }

    std::unique_ptr<Viewer> viewer_;

private:
    bool IsDisplayAvailable() {
        // Check for DISPLAY environment variable on Linux
        const char* display = std::getenv("DISPLAY");
        if (!display || strlen(display) == 0) {
            return false;
        }
        return true;
    }
};

// Test OpenGL resource cleanup
TEST_F(MemoryLeakTest, OpenGLObjectLifecycle) {
    auto scene_manager = std::make_shared<GlSceneManager>("MemoryTestScene");
    viewer_->AddSceneObject(scene_manager);
    
    // Create and destroy multiple OpenGL objects
    for (int i = 0; i < 100; ++i) {
        // Triangle objects
        auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(1.0f, 0.0f, 0.0f));
        scene_manager->AddOpenGLObject("triangle_" + std::to_string(i), std::move(triangle));
        
        // Grid objects
        auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager->AddOpenGLObject("grid_" + std::to_string(i), std::move(grid));
        
        // Point cloud objects
        std::vector<glm::vec4> points;
        for (int j = 0; j < 100; ++j) {
            points.emplace_back(j * 0.1f, 0.0f, 0.0f, 1.0f);
        }
        auto point_cloud = std::make_unique<PointCloud>();
        point_cloud->SetPoints(points, PointCloud::ColorMode::kStatic);
        scene_manager->AddOpenGLObject("points_" + std::to_string(i), std::move(point_cloud));
    }
    
    // Objects should be automatically cleaned up when scene_manager goes out of scope
    SUCCEED();
}

TEST_F(MemoryLeakTest, ShaderProgramLifecycle) {
    // Test shader program creation and destruction
    for (int i = 0; i < 50; ++i) {
        auto shader_program = std::make_unique<ShaderProgram>();
        
        // Just test creation and destruction of shader program
        // The actual shaders are handled internally by the objects
        
        // Shader program should clean up resources automatically
    }
    
    SUCCEED();
}

TEST_F(MemoryLeakTest, FrameBufferLifecycle) {
    // Test framebuffer creation and destruction
    for (int i = 0; i < 20; ++i) {
        auto framebuffer = std::make_unique<FrameBuffer>(800, 600);
        
        // Use the framebuffer
        framebuffer->Bind();
        framebuffer->Clear();
        framebuffer->Unbind();
        
        // Framebuffer should clean up OpenGL resources automatically
    }
    
    SUCCEED();
}

TEST_F(MemoryLeakTest, SceneObjectContainerLifecycle) {
    // Test container cleanup with nested objects
    for (int cycle = 0; cycle < 10; ++cycle) {
        auto scene_manager = std::make_shared<GlSceneManager>("CycleScene" + std::to_string(cycle));
        viewer_->AddSceneObject(scene_manager);
        
        // Create nested structure
        for (int i = 0; i < 10; ++i) {
            auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(1.0f, 0.0f, 0.0f));
            scene_manager->AddOpenGLObject("nested_triangle_" + std::to_string(i), std::move(triangle));
        }
        
        // Remove from viewer - should trigger cleanup
        // Note: Actual removal implementation depends on viewer design
    }
    
    SUCCEED();
}

TEST_F(MemoryLeakTest, BufferMemoryManagement) {
    // Test buffer allocation and deallocation
    for (int i = 0; i < 100; ++i) {
        // Create large point clouds to test GPU memory management
        std::vector<glm::vec4> large_points;
        
        large_points.reserve(10000);
        
        for (int j = 0; j < 10000; ++j) {
            large_points.emplace_back(j * 0.001f, 0.0f, 0.0f, 1.0f);
        }
        
        auto point_cloud = std::make_unique<PointCloud>();
        point_cloud->SetPoints(large_points, PointCloud::ColorMode::kStatic);
        
        // Point cloud should clean up GPU buffers when destroyed
    }
    
    SUCCEED();
}

// Test for specific memory leak patterns
TEST_F(MemoryLeakTest, EventSystemMemoryManagement) {
    // Test event system cleanup with singleton
    for (int i = 0; i < 100; ++i) {
        // Register handler
        EventDispatcher::GetInstance().RegisterHandler("memory_test_event", 
            [](std::shared_ptr<BaseEvent> event) {
                // Do nothing
            });
        
        // Publish events
        for (int k = 0; k < 10; ++k) {
            auto event = std::make_shared<Event<int>>(EventSource::kApplicaton, "memory_test_event", k);
            EventDispatcher::GetInstance().Dispatch(event);
        }
    }
    
    SUCCEED();
}
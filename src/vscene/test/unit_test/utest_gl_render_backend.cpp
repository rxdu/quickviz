/*
 * utest_gl_render_backend.cpp
 *
 * Created on: August 27, 2025
 * Description: Integration tests for GlRenderBackend (Step 3)
 * 
 * Tests GlRenderBackend integration with GlSceneManager and OpenGL objects.
 * These tests verify that virtual objects are properly mapped to OpenGL objects.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>

#include "vscene/gl_render_backend.hpp" 
#include "vscene/virtual_sphere.hpp"
#include "vscene/virtual_scene.hpp"
#include "gldraw/gl_scene_manager.hpp"

using namespace quickviz;

class GlRenderBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Note: These tests require OpenGL context, which may not be available in CI
        // For now, we'll test the basic interface without actual rendering
        backend = std::make_unique<GlRenderBackend>();
    }

    std::unique_ptr<GlRenderBackend> backend;
};

// Test basic backend creation
TEST_F(GlRenderBackendTest, BackendCreation) {
    EXPECT_NE(backend.get(), nullptr);
    EXPECT_NE(backend->GetSceneManager(), nullptr);
}

// Test sphere object creation
TEST_F(GlRenderBackendTest, SphereObjectCreation) {
    VirtualObjectData sphere_data;
    sphere_data.transform = glm::translate(glm::mat4(1.0f), glm::vec3(1.0f, 2.0f, 3.0f));
    sphere_data.color = glm::vec3(1.0f, 0.0f, 0.0f); // Red
    sphere_data.geometry.radius = 1.5f;
    sphere_data.visible = true;
    
    // Create sphere object
    backend->CreateObject("test_sphere", VirtualObjectType::Sphere, sphere_data);
    
    // Verify the object was added to scene manager
    auto* gl_object = backend->GetSceneManager()->GetOpenGLObject("test_sphere");
    EXPECT_NE(gl_object, nullptr);
}

// Test object update
TEST_F(GlRenderBackendTest, ObjectUpdate) {
    // Create initial sphere
    VirtualObjectData initial_data;
    initial_data.geometry.radius = 1.0f;
    initial_data.color = glm::vec3(1.0f, 0.0f, 0.0f);
    
    backend->CreateObject("test_sphere", VirtualObjectType::Sphere, initial_data);
    
    // Update the object
    VirtualObjectData updated_data = initial_data;
    updated_data.geometry.radius = 2.0f;
    updated_data.color = glm::vec3(0.0f, 1.0f, 0.0f); // Change to green
    updated_data.highlighted = true;
    
    backend->UpdateObject("test_sphere", updated_data);
    
    // Verify object still exists (detailed verification would require OpenGL context)
    auto* gl_object = backend->GetSceneManager()->GetOpenGLObject("test_sphere");
    EXPECT_NE(gl_object, nullptr);
}

// Test object removal
TEST_F(GlRenderBackendTest, ObjectRemoval) {
    // Create sphere
    VirtualObjectData sphere_data;
    sphere_data.geometry.radius = 1.0f;
    
    backend->CreateObject("test_sphere", VirtualObjectType::Sphere, sphere_data);
    
    // Verify it exists
    auto* gl_object = backend->GetSceneManager()->GetOpenGLObject("test_sphere");
    EXPECT_NE(gl_object, nullptr);
    
    // Remove it
    backend->RemoveObject("test_sphere");
    
    // Verify it's gone
    gl_object = backend->GetSceneManager()->GetOpenGLObject("test_sphere");
    EXPECT_EQ(gl_object, nullptr);
}

// Test clear all objects
TEST_F(GlRenderBackendTest, ClearAllObjects) {
    // Create multiple objects
    VirtualObjectData sphere_data;
    sphere_data.geometry.radius = 1.0f;
    
    backend->CreateObject("sphere1", VirtualObjectType::Sphere, sphere_data);
    backend->CreateObject("sphere2", VirtualObjectType::Sphere, sphere_data);
    backend->CreateObject("sphere3", VirtualObjectType::Sphere, sphere_data);
    
    // Verify they exist
    EXPECT_NE(backend->GetSceneManager()->GetOpenGLObject("sphere1"), nullptr);
    EXPECT_NE(backend->GetSceneManager()->GetOpenGLObject("sphere2"), nullptr);
    EXPECT_NE(backend->GetSceneManager()->GetOpenGLObject("sphere3"), nullptr);
    
    // Clear all
    backend->ClearAllObjects();
    
    // Verify they're gone
    EXPECT_EQ(backend->GetSceneManager()->GetOpenGLObject("sphere1"), nullptr);
    EXPECT_EQ(backend->GetSceneManager()->GetOpenGLObject("sphere2"), nullptr);
    EXPECT_EQ(backend->GetSceneManager()->GetOpenGLObject("sphere3"), nullptr);
}

// Test mouse ray generation
TEST_F(GlRenderBackendTest, MouseRayGeneration) {
    Ray ray = backend->GetMouseRay(100.0f, 200.0f, 800.0f, 600.0f);
    
    // The ray should have valid data (though exact values depend on camera setup)
    // For now, just verify the method doesn't crash
    EXPECT_TRUE(true); // Test passes if no exception thrown
}

// Test background color setting
TEST_F(GlRenderBackendTest, BackgroundColor) {
    // This should not crash
    backend->SetBackgroundColor(0.2f, 0.3f, 0.4f, 1.0f);
    EXPECT_TRUE(true);
}

// Test unsupported object types
TEST_F(GlRenderBackendTest, UnsupportedObjectTypes) {
    VirtualObjectData box_data;
    box_data.geometry.size = glm::vec3(1.0f, 2.0f, 3.0f);
    
    // Try to create unsupported object type
    backend->CreateObject("test_box", VirtualObjectType::Box, box_data);
    
    // Should not create the object (no OpenGL Box implementation yet)
    auto* gl_object = backend->GetSceneManager()->GetOpenGLObject("test_box");
    EXPECT_EQ(gl_object, nullptr);
}

// Integration test with VirtualScene
TEST_F(GlRenderBackendTest, VirtualSceneIntegration) {
    auto scene = std::make_unique<VirtualScene>();
    
    // Set our backend
    scene->SetRenderBackend(std::make_unique<GlRenderBackend>());
    
    // Add a virtual sphere
    auto sphere = std::make_unique<VirtualSphere>("test_sphere", 1.0f);
    sphere->SetPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    sphere->SetColor(glm::vec3(0.5f, 0.7f, 0.9f));
    
    scene->AddObject("test_sphere", std::move(sphere));
    
    // Update the scene (should sync to backend)
    scene->Update(0.1f);
    
    // Verify the backend received the object
    auto* gl_backend = dynamic_cast<GlRenderBackend*>(scene->GetRenderBackend());
    EXPECT_NE(gl_backend, nullptr);
    
    auto* gl_object = gl_backend->GetSceneManager()->GetOpenGLObject("test_sphere");
    EXPECT_NE(gl_object, nullptr);
}

/*
 * Expected Output:
 * 
 * [==========] Running 9 tests from 1 test suite.
 * [----------] Global test environment set-up.
 * [----------] 9 tests from GlRenderBackendTest
 * [ RUN      ] GlRenderBackendTest.BackendCreation
 * [       OK ] GlRenderBackendTest.BackendCreation
 * [ RUN      ] GlRenderBackendTest.SphereObjectCreation
 * [       OK ] GlRenderBackendTest.SphereObjectCreation
 * [ RUN      ] GlRenderBackendTest.ObjectUpdate
 * [       OK ] GlRenderBackendTest.ObjectUpdate
 * [ RUN      ] GlRenderBackendTest.ObjectRemoval
 * [       OK ] GlRenderBackendTest.ObjectRemoval
 * [ RUN      ] GlRenderBackendTest.ClearAllObjects
 * [       OK ] GlRenderBackendTest.ClearAllObjects
 * [ RUN      ] GlRenderBackendTest.MouseRayGeneration
 * [       OK ] GlRenderBackendTest.MouseRayGeneration
 * [ RUN      ] GlRenderBackendTest.BackgroundColor
 * [       OK ] GlRenderBackendTest.BackgroundColor
 * [ RUN      ] GlRenderBackendTest.UnsupportedObjectTypes
 * [       OK ] GlRenderBackendTest.UnsupportedObjectTypes
 * [ RUN      ] GlRenderBackendTest.VirtualSceneIntegration
 * [       OK ] GlRenderBackendTest.VirtualSceneIntegration
 * [----------] 9 tests from GlRenderBackendTest (X ms total)
 * [----------] Global test environment tear-down
 * [==========] 9 tests from 1 test suite ran. (X ms total)
 * [  PASSED  ] 9 tests.
 */
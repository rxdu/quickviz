/*
 * utest_render_backend.cpp
 *
 * Created on: August 27, 2025
 * Description: Unit tests for RenderInterface using MockRenderBackend
 * 
 * Tests render backend interface operations without OpenGL dependencies.
 * GL integration testing is handled by manual tests.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>

#include "vscene/mock_render_backend.hpp" 
#include "vscene/virtual_sphere.hpp"
#include "vscene/virtual_scene.hpp"

using namespace quickviz;

class RenderBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        backend = std::make_unique<MockRenderBackend>();
    }

    std::unique_ptr<MockRenderBackend> backend;
};

// Test basic backend creation
TEST_F(RenderBackendTest, BackendCreation) {
    EXPECT_NE(backend.get(), nullptr);
    EXPECT_EQ(backend->GetObjectCount(), 0);
}

// Test sphere object creation
TEST_F(RenderBackendTest, SphereObjectCreation) {
    VirtualObjectData sphere_data;
    sphere_data.transform = glm::translate(glm::mat4(1.0f), glm::vec3(1.0f, 2.0f, 3.0f));
    sphere_data.color = glm::vec3(1.0f, 0.0f, 0.0f); // Red
    sphere_data.geometry.radius = 1.5f;
    sphere_data.visible = true;
    
    // Create sphere object
    backend->CreateObject("test_sphere", VirtualObjectType::Sphere, sphere_data);
    
    // Verify the object was recorded
    EXPECT_TRUE(backend->HasObject("test_sphere"));
    EXPECT_EQ(backend->GetObjectType("test_sphere"), VirtualObjectType::Sphere);
    EXPECT_EQ(backend->GetObjectCount(), 1);
    
    // Check operation log
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 1);
    EXPECT_EQ(log[0], "CREATE:test_sphere:sphere");
}

// Test object update
TEST_F(RenderBackendTest, ObjectUpdate) {
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
    
    // Verify object data was updated
    VirtualObjectData retrieved_data = backend->GetObjectData("test_sphere");
    EXPECT_FLOAT_EQ(retrieved_data.geometry.radius, 2.0f);
    EXPECT_EQ(retrieved_data.color, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_TRUE(retrieved_data.highlighted);
    
    // Check operation log
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 2);
    EXPECT_EQ(log[1], "UPDATE:test_sphere");
}

// Test object removal
TEST_F(RenderBackendTest, ObjectRemoval) {
    // Create sphere
    VirtualObjectData sphere_data;
    sphere_data.geometry.radius = 1.0f;
    
    backend->CreateObject("test_sphere", VirtualObjectType::Sphere, sphere_data);
    
    // Verify it exists
    EXPECT_TRUE(backend->HasObject("test_sphere"));
    EXPECT_EQ(backend->GetObjectCount(), 1);
    
    // Remove it
    backend->RemoveObject("test_sphere");
    
    // Verify it's gone
    EXPECT_FALSE(backend->HasObject("test_sphere"));
    EXPECT_EQ(backend->GetObjectCount(), 0);
    
    // Check operation log
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 2);
    EXPECT_EQ(log[1], "REMOVE:test_sphere");
}

// Test clear all objects
TEST_F(RenderBackendTest, ClearAllObjects) {
    // Create multiple objects
    VirtualObjectData sphere_data;
    sphere_data.geometry.radius = 1.0f;
    
    backend->CreateObject("sphere1", VirtualObjectType::Sphere, sphere_data);
    backend->CreateObject("sphere2", VirtualObjectType::Sphere, sphere_data);
    backend->CreateObject("sphere3", VirtualObjectType::Sphere, sphere_data);
    
    // Verify they exist
    EXPECT_EQ(backend->GetObjectCount(), 3);
    EXPECT_TRUE(backend->HasObject("sphere1"));
    EXPECT_TRUE(backend->HasObject("sphere2"));
    EXPECT_TRUE(backend->HasObject("sphere3"));
    
    // Clear all
    backend->ClearAllObjects();
    
    // Verify they're gone
    EXPECT_EQ(backend->GetObjectCount(), 0);
    EXPECT_FALSE(backend->HasObject("sphere1"));
    EXPECT_FALSE(backend->HasObject("sphere2"));
    EXPECT_FALSE(backend->HasObject("sphere3"));
    
    // Check operation log
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 4); // 3 creates + 1 clear
    EXPECT_EQ(log[3], "CLEAR_ALL");
}

// Test render to framebuffer
TEST_F(RenderBackendTest, RenderToFramebuffer) {
    backend->RenderToFramebuffer(800.0f, 600.0f);
    
    EXPECT_EQ(backend->GetRenderCallCount(), 1);
    EXPECT_EQ(backend->GetLastRenderSize(), glm::vec2(800.0f, 600.0f));
    
    // Check operation log
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 1);
    EXPECT_EQ(log[0], "RENDER:800.000000x600.000000");
}

// Ray-casting tests removed - using GPU ID-buffer selection exclusively

// Test object picking
TEST_F(RenderBackendTest, ObjectPicking) {
    // Set up mock picking result
    backend->SetMockPickedObject("test_sphere");
    
    std::string picked = backend->PickObjectAt(100.0f, 200.0f);
    
    EXPECT_EQ(picked, "test_sphere");
    EXPECT_EQ(backend->GetLastPickPosition(), glm::vec2(100.0f, 200.0f));
    
    // Check operation log
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 1);
    EXPECT_EQ(log[0], "PICK:100.000000,200.000000");
}

// Test background color setting
TEST_F(RenderBackendTest, BackgroundColor) {
    backend->SetBackgroundColor(0.2f, 0.3f, 0.4f, 1.0f);
    
    // Check operation log
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 1);
    EXPECT_EQ(log[0], "SET_BG_COLOR:0.200000,0.300000,0.400000,1.000000");
}

// Test framebuffer texture ID
TEST_F(RenderBackendTest, FramebufferTexture) {
    uint32_t texture_id = backend->GetFramebufferTexture();
    EXPECT_EQ(texture_id, 0); // Mock returns 0
}

// Integration test with VirtualScene
TEST_F(RenderBackendTest, VirtualSceneIntegration) {
    auto scene = std::make_unique<VirtualScene>();
    
    // Create a mock backend and keep reference
    auto backend_ptr = std::make_unique<MockRenderBackend>();
    MockRenderBackend* mock_backend = backend_ptr.get();
    
    // Set our backend
    scene->SetRenderBackend(std::move(backend_ptr));
    
    // Add a virtual sphere
    auto sphere = std::make_unique<VirtualSphere>("test_sphere", 1.0f);
    sphere->SetPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    sphere->SetColor(glm::vec3(0.5f, 0.7f, 0.9f));
    
    scene->AddObject("test_sphere", std::move(sphere));
    
    // Update the scene (should sync to backend)
    scene->Update(0.1f);
    
    // Verify the backend received the object
    EXPECT_TRUE(mock_backend->HasObject("test_sphere"));
    EXPECT_EQ(mock_backend->GetObjectType("test_sphere"), VirtualObjectType::Sphere);
    
    VirtualObjectData data = mock_backend->GetObjectData("test_sphere");
    EXPECT_FLOAT_EQ(data.geometry.radius, 1.0f);
    EXPECT_EQ(data.color, glm::vec3(0.5f, 0.7f, 0.9f));
}

// Test operation log utilities
TEST_F(RenderBackendTest, LogUtilities) {
    backend->CreateObject("obj1", VirtualObjectType::Sphere, VirtualObjectData{});
    backend->UpdateObject("obj1", VirtualObjectData{});
    backend->RemoveObject("obj1");
    
    auto& log = backend->GetOperationLog();
    EXPECT_EQ(log.size(), 3);
    
    backend->ClearLog();
    EXPECT_EQ(backend->GetOperationLog().size(), 0);
    
    // Test reset functionality
    backend->CreateObject("obj2", VirtualObjectType::Sphere, VirtualObjectData{});
    backend->Reset();
    
    EXPECT_EQ(backend->GetObjectCount(), 0);
    EXPECT_EQ(backend->GetOperationLog().size(), 0);
    EXPECT_EQ(backend->GetRenderCallCount(), 0);
}
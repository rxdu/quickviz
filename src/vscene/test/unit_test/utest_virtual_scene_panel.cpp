/*
 * utest_virtual_scene_panel.cpp
 *
 * Created on: August 27, 2025
 * Description: Unit tests for VirtualScenePanel
 * 
 * Tests the panel integration layer without requiring OpenGL context.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>

#include "vscene/virtual_scene_panel.hpp"
#include "vscene/virtual_sphere.hpp"
#include "vscene/mock_render_backend.hpp"

using namespace quickviz;

class VirtualScenePanelTest : public ::testing::Test {
protected:
    void SetUp() override {
        panel_ = std::make_unique<VirtualScenePanel>("Test Panel");
        
        // Set up mock backend
        auto backend = std::make_unique<MockRenderBackend>();
        backend_ = backend.get(); // Keep reference for testing
        panel_->SetRenderBackend(std::move(backend));
    }

    std::unique_ptr<VirtualScenePanel> panel_;
    MockRenderBackend* backend_; // Non-owning reference
};

// Test basic panel creation and access
TEST_F(VirtualScenePanelTest, BasicCreationAndAccess) {
    EXPECT_NE(panel_->GetVirtualScene(), nullptr);
    EXPECT_NE(panel_->GetVirtualEventDispatcher(), nullptr);
    EXPECT_EQ(panel_->GetRenderBackend(), backend_);
    
    // Test initial state
    EXPECT_EQ(panel_->GetVirtualScene()->GetObjectIds().size(), 0);
    EXPECT_EQ(panel_->GetSelectedIds().size(), 0);
}

// Test configuration
TEST_F(VirtualScenePanelTest, Configuration) {
    VirtualScenePanel::Config config;
    config.show_selection_outline = false;
    config.enable_multi_selection = false;
    config.show_debug_info = true;
    config.selection_color = glm::vec3(1.0f, 0.0f, 0.0f); // Red
    
    panel_->SetConfig(config);
    
    const auto& retrieved_config = panel_->GetConfig();
    EXPECT_FALSE(retrieved_config.show_selection_outline);
    EXPECT_FALSE(retrieved_config.enable_multi_selection);
    EXPECT_TRUE(retrieved_config.show_debug_info);
    EXPECT_EQ(retrieved_config.selection_color, glm::vec3(1.0f, 0.0f, 0.0f));
}

// Test object management delegation
TEST_F(VirtualScenePanelTest, ObjectManagement) {
    // Create and add object
    auto sphere = CreateVirtualSphere("test_sphere", 1.5f);
    sphere->SetPosition(glm::vec3(1, 2, 3));
    sphere->SetColor(glm::vec3(0, 1, 0));
    
    panel_->AddObject("test_sphere", std::move(sphere));
    
    // Verify object was added
    EXPECT_EQ(panel_->GetVirtualScene()->GetObjectIds().size(), 1);
    
    auto* retrieved_object = panel_->GetObject("test_sphere");
    EXPECT_NE(retrieved_object, nullptr);
    EXPECT_EQ(retrieved_object->GetId(), "test_sphere");
    
    // Verify backend received the creation call
    EXPECT_EQ(backend_->GetObjectCount(), 1);
    auto object_type = backend_->GetObjectType("test_sphere");
    EXPECT_EQ(object_type, VirtualObjectType::Sphere);
}

// Test selection delegation
TEST_F(VirtualScenePanelTest, SelectionManagement) {
    // Add objects
    panel_->AddObject("sphere1", CreateVirtualSphere("sphere1", 1.0f));
    panel_->AddObject("sphere2", CreateVirtualSphere("sphere2", 1.0f));
    panel_->AddObject("sphere3", CreateVirtualSphere("sphere3", 1.0f));
    
    // Test selection through scene
    panel_->GetVirtualScene()->SetSelected("sphere1", true);
    panel_->GetVirtualScene()->SetSelected("sphere2", true);
    
    auto selected_ids = panel_->GetSelectedIds();
    EXPECT_EQ(selected_ids.size(), 2);
    EXPECT_TRUE(std::find(selected_ids.begin(), selected_ids.end(), "sphere1") != selected_ids.end());
    EXPECT_TRUE(std::find(selected_ids.begin(), selected_ids.end(), "sphere2") != selected_ids.end());
}

// Test event system access
TEST_F(VirtualScenePanelTest, EventSystemAccess) {
    std::vector<VirtualEvent> events_received;
    
    // Subscribe to events through panel
    auto* dispatcher = panel_->GetVirtualEventDispatcher();
    dispatcher->Subscribe(VirtualEventType::ObjectAdded, 
        [&events_received](const VirtualEvent& e) {
            events_received.push_back(e);
        });
    
    // Add object and verify event
    panel_->AddObject("test_sphere", CreateVirtualSphere("test_sphere", 1.0f));
    
    EXPECT_EQ(events_received.size(), 1);
    EXPECT_EQ(events_received[0].type, VirtualEventType::ObjectAdded);
    EXPECT_EQ(events_received[0].object_id, "test_sphere");
}

// Test render backend management
TEST_F(VirtualScenePanelTest, RenderBackendManagement) {
    // Initial backend should be set
    EXPECT_EQ(panel_->GetRenderBackend(), backend_);
    
    // Replace with new backend
    auto new_backend = std::make_unique<MockRenderBackend>();
    auto* new_backend_ptr = new_backend.get();
    
    panel_->SetRenderBackend(std::move(new_backend));
    EXPECT_EQ(panel_->GetRenderBackend(), new_backend_ptr);
}

// Test RenderInsideWindow doesn't crash without ImGui context
TEST_F(VirtualScenePanelTest, RenderWithoutImGui) {
    // This test verifies that RenderInsideWindow can be called without crashing
    // even when ImGui context is not available (it will just not render anything)
    
    // Add an object first
    panel_->AddObject("test_sphere", CreateVirtualSphere("test_sphere", 1.0f));
    
    // This should not crash, even without ImGui context
    // Note: In a real test environment with ImGui, this would actually render
    EXPECT_NO_THROW(panel_->RenderInsideWindow());
}

// Test helper methods
TEST_F(VirtualScenePanelTest, HelperMethods) {
    // Test convenience methods exist and work
    EXPECT_EQ(panel_->GetObject("non_existent"), nullptr);
    
    // Add object and test retrieval
    panel_->AddObject("helper_test", CreateVirtualSphere("helper_test", 2.0f));
    
    auto* object = panel_->GetObject("helper_test");
    EXPECT_NE(object, nullptr);
    EXPECT_EQ(object->GetId(), "helper_test");
    
    // Test empty selection initially
    EXPECT_EQ(panel_->GetSelectedIds().size(), 0);
}
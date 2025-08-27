/*
 * utest_virtual_scene.cpp
 *
 * Created on: August 27, 2025
 * Description: Unit tests for VirtualScene core functionality (Step 2)
 * 
 * Tests VirtualScene object management, selection, and backend integration
 * using MockRenderBackend to avoid OpenGL dependencies.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>

#include "vscene/virtual_scene.hpp"
#include "vscene/virtual_sphere.hpp"
#include "vscene/mock_render_backend.hpp"

using namespace quickviz;

class VirtualSceneTest : public ::testing::Test {
protected:
    void SetUp() override {
        scene = std::make_unique<VirtualScene>();
        
        // Create mock backend and get raw pointer before transferring ownership
        auto backend_ptr = std::make_unique<MockRenderBackend>();
        mock_backend = backend_ptr.get(); // Keep raw pointer for testing
        
        // Transfer ownership to scene
        scene->SetRenderBackend(std::move(backend_ptr));
    }

    std::unique_ptr<VirtualScene> scene;
    MockRenderBackend* mock_backend; // Raw pointer for testing access (owned by scene)
};

// Test basic scene creation and backend setup
TEST_F(VirtualSceneTest, BasicSceneCreation) {
    EXPECT_NE(scene->GetRenderBackend(), nullptr);
    EXPECT_EQ(scene->GetObjectIds().size(), 0);
    EXPECT_EQ(scene->GetSelectedCount(), 0);
}

// Test object addition and removal
TEST_F(VirtualSceneTest, ObjectManagement) {
    // Add a sphere
    auto sphere = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    scene->AddObject("sphere1", std::move(sphere));
    
    // Verify object was added
    EXPECT_EQ(scene->GetObjectIds().size(), 1);
    EXPECT_NE(scene->GetObject("sphere1"), nullptr);
    EXPECT_EQ(scene->GetObject("sphere1")->GetId(), "sphere1");
    EXPECT_EQ(scene->GetObject("sphere1")->GetType(), VirtualObjectType::Sphere);
    
    // Verify backend was notified
    EXPECT_TRUE(mock_backend->HasObject("sphere1"));
    EXPECT_EQ(mock_backend->GetObjectType("sphere1"), VirtualObjectType::Sphere);
    EXPECT_EQ(mock_backend->GetObjectCount(), 1);
    
    // Remove object
    scene->RemoveObject("sphere1");
    EXPECT_EQ(scene->GetObjectIds().size(), 0);
    EXPECT_EQ(scene->GetObject("sphere1"), nullptr);
    EXPECT_FALSE(mock_backend->HasObject("sphere1"));
}

// Test multiple objects
TEST_F(VirtualSceneTest, MultipleObjects) {
    // Add multiple spheres
    scene->AddObject("sphere1", std::make_unique<VirtualSphere>("sphere1", 1.0f));
    scene->AddObject("sphere2", std::make_unique<VirtualSphere>("sphere2", 2.0f));
    scene->AddObject("sphere3", std::make_unique<VirtualSphere>("sphere3", 0.5f));
    
    EXPECT_EQ(scene->GetObjectIds().size(), 3);
    EXPECT_EQ(mock_backend->GetObjectCount(), 3);
    
    // Check all objects exist
    EXPECT_NE(scene->GetObject("sphere1"), nullptr);
    EXPECT_NE(scene->GetObject("sphere2"), nullptr);
    EXPECT_NE(scene->GetObject("sphere3"), nullptr);
    
    // Clear all objects
    scene->ClearObjects();
    EXPECT_EQ(scene->GetObjectIds().size(), 0);
    EXPECT_EQ(mock_backend->GetObjectCount(), 0);
}

// Test selection management
TEST_F(VirtualSceneTest, SelectionManagement) {
    // Add objects
    scene->AddObject("sphere1", std::make_unique<VirtualSphere>("sphere1", 1.0f));
    scene->AddObject("sphere2", std::make_unique<VirtualSphere>("sphere2", 2.0f));
    
    // Test single selection
    scene->SetSelected("sphere1", true);
    EXPECT_TRUE(scene->IsSelected("sphere1"));
    EXPECT_FALSE(scene->IsSelected("sphere2"));
    EXPECT_EQ(scene->GetSelectedCount(), 1);
    
    auto selected_ids = scene->GetSelectedIds();
    EXPECT_EQ(selected_ids.size(), 1);
    EXPECT_EQ(selected_ids[0], "sphere1");
    
    // Test object state was updated
    EXPECT_TRUE(scene->GetObject("sphere1")->GetState().selected);
    EXPECT_FALSE(scene->GetObject("sphere2")->GetState().selected);
    
    // Test deselection
    scene->SetSelected("sphere1", false);
    EXPECT_FALSE(scene->IsSelected("sphere1"));
    EXPECT_EQ(scene->GetSelectedCount(), 0);
}

// Test multi-selection
TEST_F(VirtualSceneTest, MultiSelection) {
    // Enable multi-selection
    VirtualScene::Config config;
    config.multi_selection_enabled = true;
    scene->SetConfig(config);
    
    // Add objects
    scene->AddObject("sphere1", std::make_unique<VirtualSphere>("sphere1", 1.0f));
    scene->AddObject("sphere2", std::make_unique<VirtualSphere>("sphere2", 2.0f));
    scene->AddObject("sphere3", std::make_unique<VirtualSphere>("sphere3", 0.5f));
    
    // Select multiple objects
    scene->AddToSelection("sphere1");
    scene->AddToSelection("sphere2");
    EXPECT_EQ(scene->GetSelectedCount(), 2);
    EXPECT_TRUE(scene->IsSelected("sphere1"));
    EXPECT_TRUE(scene->IsSelected("sphere2"));
    EXPECT_FALSE(scene->IsSelected("sphere3"));
    
    // Test SelectAll
    scene->SelectAll();
    EXPECT_EQ(scene->GetSelectedCount(), 3);
    
    // Test ClearSelection
    scene->ClearSelection();
    EXPECT_EQ(scene->GetSelectedCount(), 0);
}

// Test selection toggle
TEST_F(VirtualSceneTest, SelectionToggle) {
    scene->AddObject("sphere1", std::make_unique<VirtualSphere>("sphere1", 1.0f));
    
    // Toggle selection on
    scene->ToggleSelection("sphere1");
    EXPECT_TRUE(scene->IsSelected("sphere1"));
    
    // Toggle selection off
    scene->ToggleSelection("sphere1");
    EXPECT_FALSE(scene->IsSelected("sphere1"));
}

// Test picking functionality
TEST_F(VirtualSceneTest, PickingFunctionality) {
    scene->AddObject("sphere1", std::make_unique<VirtualSphere>("sphere1", 1.0f));
    
    // Set up mock to return specific object when picked
    mock_backend->SetMockPickedObject("sphere1");
    
    // Test picking
    VirtualObject* picked = scene->Pick(100.0f, 150.0f);
    EXPECT_NE(picked, nullptr);
    EXPECT_EQ(picked->GetId(), "sphere1");
    
    // Verify backend was called with correct coordinates
    EXPECT_EQ(mock_backend->GetLastPickPosition(), glm::vec2(100.0f, 150.0f));
}

// Test scene bounds calculation
TEST_F(VirtualSceneTest, SceneBounds) {
    // Empty scene bounds
    BoundingBox empty_bounds = scene->GetSceneBounds();
    EXPECT_EQ(empty_bounds.min, glm::vec3(0.0f));
    EXPECT_EQ(empty_bounds.max, glm::vec3(0.0f));
    
    // Add positioned spheres
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    sphere1->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    scene->AddObject("sphere1", std::move(sphere1));
    
    auto sphere2 = std::make_unique<VirtualSphere>("sphere2", 0.5f);
    sphere2->SetPosition(glm::vec3(5.0f, 5.0f, 5.0f));
    scene->AddObject("sphere2", std::move(sphere2));
    
    // Calculate scene bounds
    BoundingBox scene_bounds = scene->GetSceneBounds();
    
    // Should encompass both spheres
    EXPECT_LE(scene_bounds.min.x, -1.0f); // sphere1 extends to -1
    EXPECT_LE(scene_bounds.min.y, -1.0f);
    EXPECT_LE(scene_bounds.min.z, -1.0f);
    EXPECT_GE(scene_bounds.max.x, 5.5f);  // sphere2 extends to 5.5
    EXPECT_GE(scene_bounds.max.y, 5.5f);
    EXPECT_GE(scene_bounds.max.z, 5.5f);
}

// Test selection bounds and centroid
TEST_F(VirtualSceneTest, SelectionBounds) {
    // Add positioned spheres
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    sphere1->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    scene->AddObject("sphere1", std::move(sphere1));
    
    auto sphere2 = std::make_unique<VirtualSphere>("sphere2", 0.5f);
    sphere2->SetPosition(glm::vec3(4.0f, 0.0f, 0.0f));
    scene->AddObject("sphere2", std::move(sphere2));
    
    // Select both objects
    VirtualScene::Config config;
    config.multi_selection_enabled = true;
    scene->SetConfig(config);
    scene->SetSelected("sphere1", true);
    scene->SetSelected("sphere2", true);
    
    // Test selection bounds
    BoundingBox selection_bounds = scene->GetSelectionBounds();
    EXPECT_FLOAT_EQ(selection_bounds.min.x, -1.0f);  // sphere1 left edge
    EXPECT_FLOAT_EQ(selection_bounds.max.x, 4.5f);   // sphere2 right edge
    
    // Test selection centroid
    glm::vec3 centroid = scene->GetSelectionCentroid();
    EXPECT_FLOAT_EQ(centroid.x, 1.75f); // Center between -1 and 4.5
    EXPECT_FLOAT_EQ(centroid.y, 0.0f);
    EXPECT_FLOAT_EQ(centroid.z, 0.0f);
}

// Test selection transform operations
TEST_F(VirtualSceneTest, SelectionTransforms) {
    // Add positioned sphere
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    sphere1->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    scene->AddObject("sphere1", std::move(sphere1));
    
    scene->SetSelected("sphere1", true);
    
    // Test translation
    scene->TranslateSelection(glm::vec3(2.0f, 1.0f, 0.0f));
    glm::vec3 new_pos = glm::vec3(scene->GetObject("sphere1")->GetState().transform[3]);
    EXPECT_FLOAT_EQ(new_pos.x, 2.0f);
    EXPECT_FLOAT_EQ(new_pos.y, 1.0f);
    EXPECT_FLOAT_EQ(new_pos.z, 0.0f);
}

// Test backend synchronization
TEST_F(VirtualSceneTest, BackendSynchronization) {
    scene->AddObject("sphere1", std::make_unique<VirtualSphere>("sphere1", 1.0f));
    
    mock_backend->ClearLog();
    
    // Modify object
    scene->GetObject("sphere1")->SetPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    
    // Update should sync to backend
    scene->Update(0.1f);
    
    // Verify backend was updated
    const auto& log = mock_backend->GetOperationLog();
    bool found_update = false;
    for (const auto& entry : log) {
        if (entry.find("UPDATE:sphere1") != std::string::npos) {
            found_update = true;
            break;
        }
    }
    EXPECT_TRUE(found_update);
}

// Test render functionality
TEST_F(VirtualSceneTest, RenderFunctionality) {
    scene->AddObject("sphere1", std::make_unique<VirtualSphere>("sphere1", 1.0f));
    
    int initial_render_count = mock_backend->GetRenderCallCount();
    
    // Render scene
    scene->Render();
    
    // Verify render was called
    EXPECT_EQ(mock_backend->GetRenderCallCount(), initial_render_count + 1);
    EXPECT_EQ(mock_backend->GetLastRenderSize(), glm::vec2(800.0f, 600.0f));
}

// Test configuration
TEST_F(VirtualSceneTest, Configuration) {
    VirtualScene::Config config;
    config.multi_selection_enabled = false;
    config.auto_update_backend = false;
    config.enable_hover_tracking = false;
    
    scene->SetConfig(config);
    const auto& retrieved_config = scene->GetConfig();
    
    EXPECT_FALSE(retrieved_config.multi_selection_enabled);
    EXPECT_FALSE(retrieved_config.auto_update_backend);
    EXPECT_FALSE(retrieved_config.enable_hover_tracking);
}

/*
 * Expected Output:
 * 
 * [==========] Running 12 tests from 1 test suite.
 * [----------] Global test environment set-up.
 * [----------] 12 tests from VirtualSceneTest
 * [ RUN      ] VirtualSceneTest.BasicSceneCreation
 * [       OK ] VirtualSceneTest.BasicSceneCreation
 * [ RUN      ] VirtualSceneTest.ObjectManagement
 * [       OK ] VirtualSceneTest.ObjectManagement
 * [ RUN      ] VirtualSceneTest.MultipleObjects
 * [       OK ] VirtualSceneTest.MultipleObjects
 * [ RUN      ] VirtualSceneTest.SelectionManagement
 * [       OK ] VirtualSceneTest.SelectionManagement
 * [ RUN      ] VirtualSceneTest.MultiSelection
 * [       OK ] VirtualSceneTest.MultiSelection
 * [ RUN      ] VirtualSceneTest.SelectionToggle
 * [       OK ] VirtualSceneTest.SelectionToggle
 * [ RUN      ] VirtualSceneTest.PickingFunctionality
 * [       OK ] VirtualSceneTest.PickingFunctionality
 * [ RUN      ] VirtualSceneTest.SceneBounds
 * [       OK ] VirtualSceneTest.SceneBounds
 * [ RUN      ] VirtualSceneTest.SelectionBounds
 * [       OK ] VirtualSceneTest.SelectionBounds
 * [ RUN      ] VirtualSceneTest.SelectionTransforms
 * [       OK ] VirtualSceneTest.SelectionTransforms
 * [ RUN      ] VirtualSceneTest.BackendSynchronization
 * [       OK ] VirtualSceneTest.BackendSynchronization
 * [ RUN      ] VirtualSceneTest.RenderFunctionality
 * [       OK ] VirtualSceneTest.RenderFunctionality
 * [ RUN      ] VirtualSceneTest.Configuration
 * [       OK ] VirtualSceneTest.Configuration
 * [----------] 12 tests from VirtualSceneTest (X ms total)
 * [----------] Global test environment tear-down.
 * [==========] 12 tests from 1 test suite ran. (X ms total)
 * [  PASSED  ] 12 tests.
 */
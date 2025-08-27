/*
 * utest_virtual_sphere_integration.cpp
 *
 * Created on: August 27, 2025
 * Description: Unit tests for VirtualSphere integration with MockRenderBackend
 * 
 * Tests the same functionality as the visual test but using MockRenderBackend
 * for fast, reliable testing without OpenGL context requirements.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>

#include "vscene/virtual_scene.hpp"
#include "vscene/virtual_sphere.hpp"
#include "vscene/mock_render_backend.hpp"
#include "vscene/virtual_object_types.hpp"

using namespace quickviz;

class VirtualSphereIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create virtual scene with MockRenderBackend
        scene_ = std::make_unique<VirtualScene>();
        backend_ = std::make_unique<MockRenderBackend>();
        backend_ptr_ = backend_.get();  // Keep raw pointer for verification
        scene_->SetRenderBackend(std::move(backend_));
    }

    void CreateTestSpheres() {
        // Create the same spheres as the visual test
        
        // 1. Basic virtual sphere - Red
        auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
        sphere1->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
        sphere1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        scene_->AddObject("sphere1", std::move(sphere1));
        
        // 2. Large virtual sphere - Green  
        auto sphere2 = std::make_unique<VirtualSphere>("sphere2", 2.0f);
        sphere2->SetPosition(glm::vec3(-4.0f, 0.0f, 0.0f));
        sphere2->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        scene_->AddObject("sphere2", std::move(sphere2));
        
        // 3. Small virtual sphere - Cyan
        auto sphere3 = std::make_unique<VirtualSphere>("sphere3", 0.5f);
        sphere3->SetPosition(glm::vec3(3.0f, 0.0f, 0.0f));
        sphere3->SetColor(glm::vec3(0.0f, 0.8f, 1.0f));
        scene_->AddObject("sphere3", std::move(sphere3));
        
        // 4. Virtual sphere - Yellow (transparency not implemented in Step 3)
        auto sphere4 = std::make_unique<VirtualSphere>("sphere4", 1.5f);
        sphere4->SetPosition(glm::vec3(0.0f, 3.0f, 0.0f));
        sphere4->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        scene_->AddObject("sphere4", std::move(sphere4));
        
        // 5. Selected virtual sphere - Magenta
        auto sphere5 = std::make_unique<VirtualSphere>("sphere5", 1.2f);
        sphere5->SetPosition(glm::vec3(0.0f, -3.0f, 0.0f));
        sphere5->SetColor(glm::vec3(1.0f, 0.0f, 1.0f));
        scene_->AddObject("sphere5", std::move(sphere5));
        
        // 6. Hidden virtual sphere - Blue
        auto sphere6 = std::make_unique<VirtualSphere>("sphere6", 0.8f);
        sphere6->SetPosition(glm::vec3(2.0f, 2.0f, 0.0f));
        sphere6->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
        sphere6->SetVisible(false);  // Initially hidden
        scene_->AddObject("sphere6", std::move(sphere6));

        // Select sphere5 using scene selection system
        scene_->SetSelected("sphere5", true);
        
        // Update scene to sync with backend
        scene_->Update(0.0f);
    }

    std::unique_ptr<VirtualScene> scene_;
    std::unique_ptr<MockRenderBackend> backend_;
    MockRenderBackend* backend_ptr_;
};

// Test basic scene setup and object creation
TEST_F(VirtualSphereIntegrationTest, SceneCreation) {
    CreateTestSpheres();
    
    // Verify all objects were added to the scene
    auto object_ids = scene_->GetObjectIds();
    EXPECT_EQ(object_ids.size(), 6);
    
    // Verify scene contains expected object IDs
    std::vector<std::string> expected_ids = {
        "sphere1", "sphere2", "sphere3", "sphere4", "sphere5", "sphere6"
    };
    for (const auto& id : expected_ids) {
        EXPECT_TRUE(std::find(object_ids.begin(), object_ids.end(), id) != object_ids.end());
    }
}

// Test backend object creation
TEST_F(VirtualSphereIntegrationTest, BackendObjectCreation) {
    CreateTestSpheres();
    
    // Verify all objects were created in the backend
    EXPECT_EQ(backend_ptr_->GetObjectCount(), 6);
    
    // Check that all objects are spheres
    std::vector<std::string> sphere_ids = {"sphere1", "sphere2", "sphere3", "sphere4", "sphere5", "sphere6"};
    for (const auto& id : sphere_ids) {
        EXPECT_TRUE(backend_ptr_->HasObject(id));
        EXPECT_EQ(backend_ptr_->GetObjectType(id), VirtualObjectType::Sphere);
    }
    
    // Verify specific object properties
    
    // sphere1: Red basic sphere at origin
    auto sphere1_data = backend_ptr_->GetObjectData("sphere1");
    EXPECT_FLOAT_EQ(sphere1_data.geometry.radius, 1.0f);
    EXPECT_EQ(sphere1_data.color, glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_EQ(glm::vec3(sphere1_data.transform[3]), glm::vec3(0.0f, 0.0f, 0.0f));
    
    // sphere2: Green large sphere  
    auto sphere2_data = backend_ptr_->GetObjectData("sphere2");
    EXPECT_FLOAT_EQ(sphere2_data.geometry.radius, 2.0f);
    EXPECT_EQ(sphere2_data.color, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_EQ(glm::vec3(sphere2_data.transform[3]), glm::vec3(-4.0f, 0.0f, 0.0f));
    
    // sphere3: Cyan small sphere
    auto sphere3_data = backend_ptr_->GetObjectData("sphere3");
    EXPECT_FLOAT_EQ(sphere3_data.geometry.radius, 0.5f);
    EXPECT_EQ(sphere3_data.color, glm::vec3(0.0f, 0.8f, 1.0f));
    EXPECT_EQ(glm::vec3(sphere3_data.transform[3]), glm::vec3(3.0f, 0.0f, 0.0f));
}

// Test visibility states
TEST_F(VirtualSphereIntegrationTest, VisibilityStates) {
    CreateTestSpheres();
    
    // Most spheres should be visible
    EXPECT_TRUE(backend_ptr_->GetObjectData("sphere1").visible);
    EXPECT_TRUE(backend_ptr_->GetObjectData("sphere2").visible);
    EXPECT_TRUE(backend_ptr_->GetObjectData("sphere3").visible);
    EXPECT_TRUE(backend_ptr_->GetObjectData("sphere4").visible);
    EXPECT_TRUE(backend_ptr_->GetObjectData("sphere5").visible);
    
    // sphere6 should be hidden
    EXPECT_FALSE(backend_ptr_->GetObjectData("sphere6").visible);
}

// Test selection states  
TEST_F(VirtualSphereIntegrationTest, SelectionStates) {
    CreateTestSpheres();
    
    // Verify scene selection state
    EXPECT_TRUE(scene_->IsSelected("sphere5"));
    EXPECT_FALSE(scene_->IsSelected("sphere1"));
    EXPECT_FALSE(scene_->IsSelected("sphere2"));
    EXPECT_FALSE(scene_->IsSelected("sphere3"));
    EXPECT_FALSE(scene_->IsSelected("sphere4"));
    EXPECT_FALSE(scene_->IsSelected("sphere6"));
    
    EXPECT_EQ(scene_->GetSelectedCount(), 1);
    
    auto selected_ids = scene_->GetSelectedIds();
    EXPECT_EQ(selected_ids.size(), 1);
    EXPECT_EQ(selected_ids[0], "sphere5");
}

// Test dynamic visibility changes
TEST_F(VirtualSphereIntegrationTest, DynamicVisibilityChange) {
    CreateTestSpheres();
    
    // Initially sphere6 should be hidden
    EXPECT_FALSE(backend_ptr_->GetObjectData("sphere6").visible);
    
    // Clear operation log to track new operations
    backend_ptr_->ClearLog();
    
    // Make sphere6 visible
    auto* sphere6 = scene_->GetObject("sphere6");
    ASSERT_NE(sphere6, nullptr);
    sphere6->SetVisible(true);
    scene_->Update(0.0f);  // Sync with backend
    
    // Verify update operation was called
    auto log = backend_ptr_->GetOperationLog();
    bool found_update = false;
    for (const auto& op : log) {
        if (op.find("UPDATE:sphere6") != std::string::npos) {
            found_update = true;
            break;
        }
    }
    EXPECT_TRUE(found_update);
    
    // Verify sphere6 is now visible in backend
    EXPECT_TRUE(backend_ptr_->GetObjectData("sphere6").visible);
}

// Test object removal
TEST_F(VirtualSphereIntegrationTest, ObjectRemoval) {
    CreateTestSpheres();
    
    EXPECT_EQ(scene_->GetObjectIds().size(), 6);
    EXPECT_EQ(backend_ptr_->GetObjectCount(), 6);
    
    // Clear operation log
    backend_ptr_->ClearLog();
    
    // Remove sphere3
    scene_->RemoveObject("sphere3");
    
    // Verify object removed from scene
    EXPECT_EQ(scene_->GetObjectIds().size(), 5);
    EXPECT_EQ(scene_->GetObject("sphere3"), nullptr);
    
    // Verify removal operation was logged
    auto log = backend_ptr_->GetOperationLog();
    bool found_remove = false;
    for (const auto& op : log) {
        if (op.find("REMOVE:sphere3") != std::string::npos) {
            found_remove = true;
            break;
        }
    }
    EXPECT_TRUE(found_remove);
}

// Test scene clearing
TEST_F(VirtualSphereIntegrationTest, SceneClearing) {
    CreateTestSpheres();
    
    EXPECT_EQ(scene_->GetObjectIds().size(), 6);
    
    // Clear operation log
    backend_ptr_->ClearLog();
    
    // Clear all objects
    scene_->ClearObjects();
    
    // Verify scene is empty
    EXPECT_EQ(scene_->GetObjectIds().size(), 0);
    
    // Verify remove operations were logged for each object
    auto log = backend_ptr_->GetOperationLog();
    
    // Count REMOVE operations - should be 6 (one for each sphere)
    int remove_count = 0;
    for (const auto& op : log) {
        if (op.find("REMOVE:") != std::string::npos) {
            remove_count++;
        }
    }
    EXPECT_EQ(remove_count, 6);  // Should have 6 remove operations
}

// Test multi-selection operations
TEST_F(VirtualSphereIntegrationTest, MultiSelection) {
    CreateTestSpheres();
    
    // Initially only sphere5 is selected
    EXPECT_EQ(scene_->GetSelectedCount(), 1);
    
    // Add sphere1 and sphere3 to selection
    scene_->AddToSelection("sphere1");
    scene_->AddToSelection("sphere3");
    
    EXPECT_EQ(scene_->GetSelectedCount(), 3);
    EXPECT_TRUE(scene_->IsSelected("sphere1"));
    EXPECT_TRUE(scene_->IsSelected("sphere3"));
    EXPECT_TRUE(scene_->IsSelected("sphere5"));
    
    // Toggle sphere5 (should remove it)
    scene_->ToggleSelection("sphere5");
    
    EXPECT_EQ(scene_->GetSelectedCount(), 2);
    EXPECT_TRUE(scene_->IsSelected("sphere1"));
    EXPECT_TRUE(scene_->IsSelected("sphere3"));
    EXPECT_FALSE(scene_->IsSelected("sphere5"));
    
    // Clear all selection
    scene_->ClearSelection();
    EXPECT_EQ(scene_->GetSelectedCount(), 0);
}

// Test scene bounds calculation
TEST_F(VirtualSphereIntegrationTest, SceneBounds) {
    CreateTestSpheres();
    
    auto bounds = scene_->GetSceneBounds();
    
    // With spheres at various positions and sizes, check reasonable bounds
    // sphere2 at (-4,0,0) with radius 2.0 should extend to -6 on x-axis
    // sphere3 at (3,0,0) with radius 0.5 should extend to 3.5 on x-axis
    // sphere4 at (0,3,0) with radius 1.5 should extend to 4.5 on y-axis
    // sphere5 at (0,-3,0) with radius 1.2 should extend to -4.2 on y-axis
    
    EXPECT_LE(bounds.min.x, -5.5f);  // At least to sphere2's extent
    EXPECT_GE(bounds.max.x, 3.2f);   // At least to sphere3's extent
    EXPECT_LE(bounds.min.y, -4.0f);  // At least to sphere5's extent  
    EXPECT_GE(bounds.max.y, 4.0f);   // At least to sphere4's extent
}

// Test that backend integration maintains virtual object properties
TEST_F(VirtualSphereIntegrationTest, VirtualObjectPropertyPreservation) {
    CreateTestSpheres();
    
    // Get virtual object
    auto* sphere1 = dynamic_cast<VirtualSphere*>(scene_->GetObject("sphere1"));
    ASSERT_NE(sphere1, nullptr);
    
    // Check virtual object properties are preserved
    EXPECT_FLOAT_EQ(sphere1->GetRadius(), 1.0f);
    EXPECT_EQ(sphere1->GetState().color, glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_EQ(sphere1->GetType(), VirtualObjectType::Sphere);
    EXPECT_EQ(std::string(sphere1->GetTypeString()), std::string("sphere"));
    
    // Verify backend data matches virtual object state
    auto backend_data = backend_ptr_->GetObjectData("sphere1");
    
    EXPECT_FLOAT_EQ(backend_data.geometry.radius, sphere1->GetRadius());
    EXPECT_EQ(backend_data.color, sphere1->GetState().color);
    EXPECT_EQ(backend_data.visible, sphere1->GetState().visible);
}